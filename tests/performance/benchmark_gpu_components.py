#!/usr/bin/env python3
"""Sequential ABBA comparisons. Keep every raw timing and failed invocation."""
import argparse, hashlib, json, subprocess, time
from pathlib import Path


def run(args):
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    records=[]
    def execute(kind,backend,workload,binary,parameters):
        number=len(records);stem=f'{number:03d}-{kind}-{workload}-{backend}'
        result_path=out/(stem+'.json');log_path=out/(stem+'.log')
        command=[str(binary),backend,*map(str,parameters)]
        if kind=='controller':command.append(str(result_path))
        begun=time.time();process=subprocess.run(command,capture_output=True,text=True)
        log_path.write_text(process.stdout+process.stderr)
        record=dict(index=number,kind=kind,backend=backend,workload=workload,command=command,
                    started_unix=begun,returncode=process.returncode,log=log_path.name)
        if process.returncode==0:
            if kind=='controller':result=json.loads(result_path.read_text())
            else:
                result=json.loads(process.stdout);result_path.write_text(json.dumps(result,indent=2)+'\n')
            record['result']=result_path.name
            if kind!='controller':record['fixture_sha256']=hashlib.sha256(Path(parameters[-1]).read_bytes()).hexdigest()
        records.append(record);(out/'ledger.json').write_text(json.dumps(records,indent=2)+'\n')
        print(f'{number:03d} {kind} {workload} {backend} rc={process.returncode}',flush=True)
        if process.returncode:raise RuntimeError(f'Comparison failed; retained {log_path}')
    for workload in ('tiny','medium','dense','blocked'):
        for mode in ('reuse','rebuild'):
            for backend in ('original','cuda','cuda','original'):
                fixture=out/f'geometry-{workload}-{mode}.bin'
                execute('geometry',backend,workload+'-'+mode,args.geometry,[workload,mode,args.geometry_iterations,fixture])
        if workload in ('tiny','blocked'):
            for repeat in range(2):execute('geometry','auto',workload,args.geometry,[workload,'reuse',args.geometry_iterations,out/'geometry-tiny-auto.bin'])
    for workload in ('open','corridor','detour','blocked'):
        for backend in ('cpu','cuda','cuda','cpu'):
            execute('sampling',backend,workload,args.sampling,[workload,args.sampling_iterations,out/f'sampling-{workload}.bin'])
    for backend in ('original','cuda','cuda','original'):
        binary=args.original_controller if backend=='original' else args.controller
        execute('controller',backend,'off-plan-free',binary,[args.controller_iterations,args.nav_config])
    binaries={str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest() for p in (args.geometry,args.sampling,args.controller,args.original_controller)}
    (out/'binaries.json').write_text(json.dumps(binaries,indent=2)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('geometry','sampling','controller','original-controller','nav-config','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--geometry-iterations',type=int,default=100);p.add_argument('--sampling-iterations',type=int,default=40);p.add_argument('--controller-iterations',type=int,default=100)
    run(p.parse_args())
