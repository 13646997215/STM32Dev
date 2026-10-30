"""Reproducible local cross-build. Never contacts hardware or changes global PATH."""
from pathlib import Path
import argparse,json,os,subprocess,sys
ROOT=Path(__file__).resolve().parents[1]
BUNDLES=Path(os.environ.get('COURSE_BUNDLE_ROOT',str(ROOT.parents[1]/'environment/stm32-bundles')))
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--preset',choices=['Debug','Release','all'],default='all');a=parser.parse_args()
    folders=[BUNDLES/'gnu-tools-for-stm32/14.3.1+st.2/bin',BUNDLES/'cmake/4.4.0+st.1/bin',BUNDLES/'ninja/1.13.2+st.1/bin']
    for folder,exe in zip(folders,['arm-none-eabi-gcc.exe','cmake.exe','ninja.exe']):
        if not (folder/exe).is_file():raise SystemExit(f'Missing pinned tool: {folder/exe}')
    env=os.environ.copy();env['PATH']=os.pathsep.join(map(str,folders))+os.pathsep+env['PATH'];results=[]
    for preset in ['Debug','Release'] if a.preset=='all' else [a.preset]:
        for args in [['--preset',preset],['--build','--preset',preset,'--parallel','4']]:
            cmd=[str(folders[1]/'cmake.exe'),*args];r=subprocess.run(cmd,cwd=ROOT,env=env,capture_output=True,text=True,errors='replace')
            results.append({'command':cmd,'exit':r.returncode,'output':r.stdout+r.stderr});print(r.stdout+r.stderr)
            if r.returncode:break
        if results[-1]['exit']:break
    (ROOT/'test-results/build.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
    return int(any(r['exit'] for r in results))
if __name__=='__main__':sys.exit(main())
