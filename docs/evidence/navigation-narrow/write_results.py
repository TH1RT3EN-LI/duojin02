from pathlib import Path
import csv
import json
import math
import statistics

sim = Path('/home/th1rt3en/dev/forge/duojin02-sim')
main = sim.with_name('duojin02')
cache = sim / '.cache/navigation-narrow'
report = cache / 'report'
def read(name): return [json.loads(p.read_text()) for p in (cache / name).glob('*/result.json')]
primary = read('frozen-v7')
assert len(primary) == 78
arrived = [r for r in primary if r.get('arrival_acceptance_pass')]
max_value = lambda records, key, scale=1: max((r[key] * scale for r in records if r.get(key) is not None), default=math.nan)
min_value = lambda records, key, scale=1: min((r[key] * scale for r in records if r.get(key) is not None), default=math.nan)
online = [r for r in primary if r['case'].startswith('online_')]
known_door = [r for r in primary if r['case'] == 'corridor_0.50']
unknown_door = [r for r in primary if r['case'] == 'online_corridor_0.50']
count = lambda records: f"{sum(r.get('arrival_acceptance_pass',False) for r in records)}/{len(records)}"
passed = lambda records: [r for r in records if r.get('arrival_acceptance_pass')]
median_time = lambda records: statistics.median(r['sim_elapsed_s'] for r in passed(records)) if passed(records) else math.nan
failures = [r for r in primary if not r.get('arrival_acceptance_pass')]
errors = sum('error' in r for r in primary)
collisions = sum(r.get('collision_events', 0) for r in primary)
regression = read('regression-v7')
startup = read('startup-counterexamples-v7')
counterexample = read('online-counterexample-v7')
waypoints = read('waypoints-v7')
limited = read('cpu-limited-v7')
old = read('paired-previous-final-v4')
new = read('paired-optimized-v7')
checks = read('checks-v7')
assert [len(r) for r in [regression, startup, counterexample, waypoints, limited, old, new, checks]] == [6, 2, 2, 2, 5, 10, 10, 9]

pair_rows=[]
for case in sorted({r['case'] for r in old}):
    a=[r for r in old if r['case']==case];b=[r for r in new if r['case']==case]
    keys={r['seed'] for r in passed(a)} & {r['seed'] for r in passed(b)}
    ta=statistics.median(r['sim_elapsed_s'] for r in a if r['seed'] in keys) if keys else math.nan
    tb=statistics.median(r['sim_elapsed_s'] for r in b if r['seed'] in keys) if keys else math.nan
    pair_rows.append(f'| {case} | {count(a)} | {count(b)} | {len(keys)} | {ta:.2f} | {tb:.2f} |')

check_rows=[]
for r in sorted(checks,key=lambda r:r['case']):
    outcome='严格到达' if r.get('arrival_acceptance_pass') else '危险指令安全停车' if r.get('safety_stop_pass') else r.get('error') or '未到达，无碰撞' if not r.get('collision_events') else '发生碰撞'
    check_rows.append(f"| {r['case']} | {outcome} | {r.get('min_clearance_m',math.nan)*1000:.1f} |")

resource_rows=[]
for mode, selection in [('固定图',[r for r in primary if not r['case'].startswith('online_')]),('在线图',online)]:
    selection=[r for r in selection if r.get('node_resources')]
    rss=max(sum(v['peak_rss_mib'] for v in r['node_resources'].values()) for r in selection)
    cpu=statistics.median(sum(v['cpu_percent_one_core'] for v in r['node_resources'].values()) for r in selection)
    resource_rows.append(f'| {mode} | {rss:.1f} | {cpu:.1f}% |')

failure_text='无主矩阵失败。' if not failures else '\n'.join(f"- {r['case']}，种子 {r['seed']}：{r.get('error', 'Action status '+str(r.get('status')))}，真实终点误差 {r.get('goal_position_error_m',math.nan)*1000:.1f} mm；保留完整记录。" for r in failures)
section=f'''## 最终冻结结果

v7 主矩阵 **{count(primary)} 严格到达、{collisions} 次碰撞、{errors} 次启动错误**；在线无先验 **{count(online)}**。成功试验最大真实终点误差 **{max_value(arrived,'goal_position_error_m',1000):.1f} mm**、最大角度误差 **{max_value(arrived,'goal_yaw_error_rad',180/math.pi):.2f}°**、最大真实末速 **{max_value(arrived,'final_speed_mps'):.4f} m/s**。

{(report/'table.md').read_text()}

已知地图 50 cm 门 **{count(known_door)}**，未知地图 50 cm 门 **{count(unknown_door)}**；真实最小净距分别 **{min_value(known_door,'min_clearance_m',1000):.1f} / {min_value(unknown_door,'min_clearance_m',1000):.1f} mm**。这些门无法绕过，成功试验均检查了实际穿越。窄门旧种子回归 **{count(regression)}**，开阔零运动旧反例 **{count(startup)}**，未知窄门旧反例 **{count(counterexample)}**，NavigateThroughPoses **{count(waypoints)}**；航点测试评价最终目标，不声称每个中间点都停稳。

{failure_text}

开阔段速度 P95 的最大值 **{max_value(primary,'speed_p95_mps'):.3f} m/s**，open 成功耗时中位数 **{median_time([r for r in primary if r['case']=='open']):.2f} s**；50 cm 已知/未知门分别 **{median_time(known_door):.2f} / {median_time(unknown_door):.2f} s**。窄处自动减速，速度上限不等于全程平均速度。

两组使用同一最终评估器和种子 50/51；旧组为 main `7bc9e10` 的原算法及参数，保留其局部静态层与恢复行为，使用相同 Cyclone DDS 和启动检查。旧/新分别 **{count(old)} / {count(new)}**，失败计入分母；耗时仅比较双方均严格到达的相同种子。

| 场景 | v4 严格到达 | v7 严格到达 | 可配对成功数 | v4 耗时中位数 s | v7 耗时中位数 s |
| --- | --- | --- | --- | --- | --- |
{chr(10).join(pair_rows)}

![配对耗时与精度](evidence/navigation-narrow/performance.png)

![未知窄门真实轨迹与净距](evidence/navigation-narrow/online-door.png)

![未知折返定位与轨迹](evidence/navigation-narrow/online-chicane.png)

保护计算最坏单次 P95 **{max_value(primary,'guard_compute_p95_ms'):.2f} ms**，最大样本 **{max_value(primary,'guard_compute_max_ms'):.2f} ms**；控制指令周期最坏 P95 **{max_value(primary,'command_period_p95_ms'):.1f} ms**，共 **{sum(r.get('controller_deadline_warnings',0) for r in primary)}** 条控制 deadline 警告。雷达中断到真实速度低于 0.005 m/s 的延迟 **{min_value(primary,'scan_dropout_stop_latency_s'):.3f}–{max_value(primary,'scan_dropout_stop_latency_s'):.3f} s**。

仅统计定位、controller、planner、guard、Collision Monitor 五个进程，RSS 是各进程峰值之和，CPU 为跨试验平均占用之和的中位数，均以一个 x86 核为 100%；不包含完整 ROS 图和 Gazebo，不代表 Orin。

| 模式 | 最大峰值 RSS 之和 MiB | CPU 之和中位数 |
| --- | --- | --- |
{chr(10).join(resource_rows)}

整个 Gazebo + ROS + 评估器容器限为 **2 CPU / 4 GiB** 的最终 v7 为 **{count(limited)}**，碰撞 **{sum(r.get('collision_events',0) for r in limited)}** 次。这是桌面资源限额压力证据，不是 Orin 原生验收。

| 边界/故障场景 | 结果 | 真实最小净距 mm |
| --- | --- | --- |
{chr(10).join(check_rows)}

48/46/44 cm 的裸底盘可能仍有通路，当前配置未证明能可靠通过；40 cm 小于 406 mm 车宽。保护不会因为未到达而放宽。本次仍保留历史高并发零运动和启动超时记录，成功诊断重跑不能证明其根因已解决。

## 验证与证据

最终 Release 七个硬件包构建通过，main Python **44/44**、sim/场景 **53/53**、C++ **3/3**、保护层故障/几何 **11/11**、地图索引 **5/5**。随机粗筛对比、噪声门居中、孤立内部柱、几何短段与实际速度不一致的反例，以及对角平移/转动的取图边界均检查。main → sim **61 个文件**哈希相同，原有 **86 个 CAD/模型文件**哈希保留；在线 GUI 健康状态单独存档。

正式最终试验 124 次（78 主矩阵、20 旧新配对、6 窄门回归、2 开阔旧反例、2 在线旧反例、2 连续航点、9 边界/故障、5 限额），原型/失败/中断另列。每个主矩阵试验的四个导航二进制哈希一致，未在试验中改变源码、配置或碰撞标准。新增分级速度记录改变了评估器字节哈希，但碰撞、时间配对与到达计算方法的 AST 核对一致。

[逐试验结果](evidence/navigation-narrow/results.json)、[CSV](evidence/navigation-narrow/results.csv)、[所有批次索引](evidence/navigation-narrow/experiment-index.json)、[最终冻结试验](evidence/navigation-narrow/frozen-trials.tar.gz)、[原型及失败](evidence/navigation-narrow/exploratory-trials.tar.gz)、[源码和评估器](evidence/navigation-narrow/source-and-harness.tar.gz)、[最终构建](evidence/navigation-narrow/final-build.tar.gz)、[平台元数据](evidence/navigation-narrow/metadata.json)、[同步校验](evidence/navigation-narrow/main-sim-sync.json)、[运行状态](evidence/navigation-narrow/gui-health.json)。复现还依赖 sim 工作区保留的 CAD 描述资产，其哈希已存档。
'''
p=main/'docs/navigation-narrow-review.md';text=p.read_text().split('## 最终冻结结果')[0].rstrip();summary=f'最终冻结 v7 **{count(primary)} 严格到达、{collisions} 次碰撞、{errors} 次启动错误**；已知/未知 50 cm 门各 **{count(known_door)} / {count(unknown_door)}**，成功最大真实终点误差 **{max_value(arrived,"goal_position_error_m",1000):.1f} mm**。本轮有限仿真达到验收目标，历史高并发反例与真机待测项仍保留。';parts=text.split('\n\n',2);text=parts[0]+'\n\n'+parts[1]+'\n\n'+summary+'\n\n'+parts[2];p.write_text(text+'\n\n'+section)
(report/'metrics.json').write_text(json.dumps(dict(primary=count(primary),collisions=collisions,startup_errors=errors,online=count(online),known_door=count(known_door),unknown_door=count(unknown_door),regression=count(regression),limited=count(limited),max_goal_error_mm=max_value(arrived,'goal_position_error_m',1000)),indent=2))
print((report/'metrics.json').read_text())
