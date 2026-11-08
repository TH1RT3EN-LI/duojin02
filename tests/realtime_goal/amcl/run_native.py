#!/usr/bin/env python3
"""Build actual PF/sensor sources into a new output directory; never uses installed libs."""
import argparse, hashlib, json, os, pathlib, subprocess
p=argparse.ArgumentParser(); p.add_argument('--source',type=pathlib.Path,required=True);p.add_argument('--output',type=pathlib.Path,required=True);p.add_argument('--cuda',action='store_true');p.add_argument('--probe',type=pathlib.Path,default=pathlib.Path(__file__).with_name('legacy_probe.cpp'));a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=False);i=a.source/'include';commands=[]
def run(cmd,name):
 commands.append(cmd);r=subprocess.run(cmd,text=True,capture_output=True)
 (a.output/(name+'.stdout')).write_text(r.stdout);(a.output/(name+'.stderr')).write_text(r.stderr)
 (a.output/'commands.json').write_text(json.dumps(commands,indent=2)+'\n')
 if r.returncode: raise RuntimeError(f'{name} exited {r.returncode}: {r.stderr}')
 return r.stdout
objects=[]
for f in ['pf/pf.c','pf/pf_kdtree.c','pf/pf_pdf.c','pf/pf_vector.c','pf/eig3.c','map/map.c']:
 src=a.source/'src'/f;obj=a.output/(src.stem+'.o');objects.append(str(obj));run(['gcc','-std=gnu11','-Wall','-Wextra','-Werror','-O2','-DHAVE_DRAND48','-I'+str(i),'-c',str(src),'-o',str(obj)],'compile_'+src.stem)
cpp=['map/map_cspace.cpp','sensors/laser/laser.cpp','sensors/laser/observation_backend.cpp','sensors/laser/likelihood_field_model_prob.cpp']
cuda=pathlib.Path(os.environ.get('DUOJIN_AMCL_CUDA_ROOT','/opt/cuda'))
extra=[]
if a.cuda:
 obj=a.output/'observation_cuda.o';objects.append(str(obj));run([str(cuda/'bin/nvcc'),'-std=c++17','-Xcompiler=-Wall,-Wextra,-Werror','-O2','-arch=sm_120','--fmad=false','-I'+str(i),'-c',str(a.source/'src/sensors/laser/observation_cuda.cu'),'-o',str(obj)],'compile_cuda');extra=['-DDUOJIN02_AMCL_HAS_CUDA','-L'+str(cuda/'lib64'),'-Wl,-rpath,'+str(cuda/'lib64'),'-lcudart_static','-ldl','-lrt','-pthread']
binary=a.output/'native_probe';run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-O2','-I'+str(i),str(a.probe)]+[str(a.source/'src'/f) for f in cpp]+objects+extra+['-lm','-o',str(binary)],'link')
results={}
for mode in (['cpu','cuda'] if a.cuda else ['cpu']): results[mode]=json.loads(run([str(binary),mode],mode))
sha=lambda f:hashlib.sha256(f.read_bytes()).hexdigest()
(a.output/'manifest.json').write_text(json.dumps({'sources':{str(f):sha(f) for f in a.source.rglob('*') if f.is_file()},'binary_sha256':sha(binary),'probe_sha256':sha(a.probe)},indent=2)+'\n')
(a.output/'results.json').write_text(json.dumps(results,indent=2)+'\n')
print(json.dumps({'output':str(a.output),'modes':list(results),'cuda_updates':results.get('cuda',{}).get('cuda_updates',0)+sum(x['cuda_updates'] for x in results.get('cuda',{}).get('duplicate_beams',[]))+results.get('cuda',{}).get('grid_plateau',{}).get('cuda_updates',0)+results.get('cuda',{}).get('merged_modes',{}).get('cuda_updates',0)}))
