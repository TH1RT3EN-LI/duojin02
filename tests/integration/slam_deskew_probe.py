#!/usr/bin/env python3
"""Real synchronous SLAM node, delayed TF, physical reference and graph persistence.

Synthetic validation only; this is not LILocBench or an accuracy-improvement claim.
"""
import argparse
import json
import math
from pathlib import Path
import signal
import struct
import subprocess
import time

import yaml
from ros_isolation import configure_isolation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--parameters", type=Path, default=Path(__file__).resolve().parents[2] /
                        "src/duojin02_bringup/config/slam_toolbox.yaml")
    args = parser.parse_args()
    if args.executable.name != "sync_slam_toolbox_node":
        parser.error("this queue-contract probe requires sync_slam_toolbox_node and its processing_status service")
    args.output.mkdir(parents=True, exist_ok=False)
    domain = configure_isolation()
    import rclpy
    from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from std_srvs.srv import Trigger
    from slam_toolbox.srv import SerializePoseGraph, DeserializePoseGraph
    rclpy.init()
    node = rclpy.create_node("slam_deskew_production_probe")
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    static_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
    dynamic = node.create_publisher(TFMessage, "/tf", qos)
    fixed = node.create_publisher(TFMessage, "/tf_static", static_qos)
    scans = node.create_publisher(LaserScan, "/scan", qos)
    poses = []
    node.create_subscription(PoseWithCovarianceStamped, "/pose", poses.append, qos)
    params = yaml.safe_load(args.parameters.read_text())
    params["slam_toolbox"]["ros__parameters"].update({
        "use_sim_time": False, "scan_motion_compensation": True,
        "scan_reference_fraction": 0.5, "scan_max_tf_gap_sec": 0.05,
        "minimum_travel_distance": 0.001, "minimum_travel_heading": 0.001,
        "minimum_time_interval": 0.0, "transform_publish_period": 0.0,
        "map_update_interval": 0.1, "min_laser_range": 0.1,
        "max_laser_range": 8.0, "resolution": 0.05,
        "scan_geometry_wait_timeout_sec": 3.0,
    })
    parameter_file = args.output / "parameters.yaml"
    parameter_file.write_text(yaml.safe_dump(params))
    report = {"status": "running", "ros_domain_id": domain,
              "scope": "production SLAM synthetic delayed-TF beam geometry and save/reload contract"}
    process = None
    log = (args.output / "node.log").open("w")

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if process is not None and process.poll() is not None:
                raise RuntimeError("SLAM exited before probe completed")
            rclpy.spin_once(node, timeout_sec=min(0.02, max(0.0, deadline-time.monotonic())))

    def stamp(msg, ns):
        msg.sec, msg.nanosec = divmod(ns, 1000000000)

    def call(kind, service, request):
        client = node.create_client(kind, service)
        try:
            if not client.wait_for_service(timeout_sec=15):
                raise RuntimeError(f"unavailable service: {service}")
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=15)
            if not future.done() or future.result() is None:
                raise RuntimeError(f"failed service: {service}")
            return future.result()
        finally:
            node.destroy_client(client)

    t0 = node.get_clock().now().nanoseconds
    def dynamic_tf(ns):
        elapsed = (ns-t0)/1e9
        tf = TransformStamped()
        stamp(tf.header.stamp, ns)
        tf.header.frame_id, tf.child_frame_id = "odom", "base_footprint"
        tf.transform.translation.x = 0.10 * elapsed
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(0.1*elapsed), math.cos(0.1*elapsed)
        dynamic.publish(TFMessage(transforms=[tf]))

    try:
        process = subprocess.Popen([str(args.executable), "--ros-args", "--params-file", str(parameter_file)],
                                   stdout=log, stderr=subprocess.STDOUT)
        spin(1.5)
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = "base_footprint", "laser"
        tf.transform.translation.x = 0.305
        tf.transform.rotation.w = 1.0
        fixed.publish(TFMessage(transforms=[tf]))
        dynamic_tf(t0)
        spin(0.3)
        expected = []
        last_tf = t0
        for frame in range(8):
            raw = t0 + 100000000 + frame*200000000
            # Up to the raw first-beam stamp only. Last-beam TF arrives after the scan.
            for ns in range(last_tf+10000000,raw+1,10000000):
                dynamic_tf(ns)
            last_tf=raw
            scan=LaserScan()
            stamp(scan.header.stamp,raw)
            scan.header.frame_id="laser"
            scan.angle_min=-math.pi/2
            scan.angle_increment=math.pi/180
            scan.angle_max=math.pi/2
            scan.time_increment=struct.unpack("f",struct.pack("f",0.05/180))[0]
            scan.scan_time=0.2
            scan.range_min,scan.range_max=0.1,8.0
            readings=[]
            for i in range(181):
                elapsed=(raw-t0)/1e9 + i*scan.time_increment
                yaw=0.2*elapsed
                x=0.1*elapsed +0.305*math.cos(yaw)
                y=0.305*math.sin(yaw)
                angle=yaw+scan.angle_min+i*scan.angle_increment
                dx,dy=math.cos(angle),math.sin(angle)
                hits=[]
                for coordinate,direction in ((x,dx),(y,dy)):
                    if abs(direction)>1e-9:
                        for wall in (-3.0,3.0):
                            distance=(wall-coordinate)/direction
                            if distance>0:hits.append(distance)
                readings.append(min(hits))
            scan.ranges=readings
            scans.publish(scan)
            spin(0.12)
            for ns in range(last_tf+10000000,raw+60000001,10000000):
                dynamic_tf(ns)
            last_tf=raw+60000000
            expected.append(raw+round(0.5*180*scan.time_increment*1e9))
            spin(0.16)
        spin(1.0)
        result=call(Trigger,"/slam_toolbox/processing_status",Trigger.Request())
        report["processing"]=json.loads(result.message)
        assert report["processing"]["completed"] == 8, report
        assert len(poses)==8, len(poses)
        actual=[pose.header.stamp.sec*1000000000+pose.header.stamp.nanosec for pose in poses]
        assert actual==expected,(actual,expected)
        prefix=args.output/"map"
        request=SerializePoseGraph.Request();request.filename=str(prefix)
        response=call(SerializePoseGraph,"/slam_toolbox/serialize_map",request)
        assert response.result==response.RESULT_SUCCESS,response
        sidecar=json.loads(prefix.with_name(prefix.name+".scan_geometry.json").read_text())
        assert sidecar["scan_motion_compensation"] and sidecar["scan_reference_fraction"]==0.5
        assert prefix.with_suffix(".data").stat().st_size>0
        # Actual graph reload with the matching contract must preserve supported geometry.
        request=DeserializePoseGraph.Request();request.filename=str(prefix)
        request.match_type=request.START_AT_FIRST_NODE
        call(DeserializePoseGraph,"/slam_toolbox/deserialize_map",request)
        report.update(status="passed", source_scan_count=8, physical_pose_stamps_ns=actual,
                      contract=sidecar, binary_save_reload=True)
        spin(0.1)
    except Exception as error:
        report.update(status="failed",error=str(error))
        raise
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=20)
        log.close()
        report["exit_code"]=process.returncode if process is not None else None
        (args.output/"validation.json").write_text(json.dumps(report,indent=2)+"\n")
        node.destroy_node()
        rclpy.shutdown()
    text=(args.output/"node.log").read_text()
    assert "scan_geometry_drop" not in text,text
    assert "Rejected scan geometry TF" not in text,text
    assert text.count("Registering sensor:") >= 2, "saved graph was not actually reloaded"
    print(json.dumps({"status":report["status"],"frames":8,"exit_code":report["exit_code"]}))


if __name__=="__main__":
    main()
