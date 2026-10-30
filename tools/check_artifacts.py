"""Inspect real ARM ELF files; never uses a programmer or a probe."""
from pathlib import Path
import json,os,re,subprocess,sys
ROOT=Path(__file__).resolve().parents[1]
BIN=Path(os.environ.get('COURSE_BUNDLE_ROOT',str(ROOT.parents[1]/'environment/stm32-bundles')))/'gnu-tools-for-stm32/14.3.1+st.2/bin'
def run(exe,*args):return subprocess.check_output([str(BIN/exe),*map(str,args)],text=True)
def main():
    results=[]
    for preset in ['Debug','Release']:
        elf=ROOT/f'build/{preset}/F401_COURSE.elf';ph=run('arm-none-eabi-readelf.exe','-lW',elf)
        symbols=run('arm-none-eabi-nm.exe','-n',elf);sizes=run('arm-none-eabi-size.exe',elf)
        failures=[]
        if ' RWE ' in ph:failures.append('ELF LOAD segment is writable and executable')
        if '0x08000000' not in ph or '0x20000000' not in ph:failures.append('Unexpected memory map')
        for name in ['App_Step','Mission_Update','Imu_Byte','Runtime_Tick','Board_Apply','DMA1_Stream5_IRQHandler','DMA2_Stream2_IRQHandler','USART1_IRQHandler','USART2_IRQHandler']:
            if not re.search(r'\b'+name+r'$',symbols,re.M):failures.append('Missing '+name)
        for name in ['DMA1_Stream5_IRQHandler','DMA2_Stream2_IRQHandler','USART1_IRQHandler','USART2_IRQHandler','EXTI9_5_IRQHandler']:
            if not re.search(r'\bT '+name+r'$',symbols,re.M):failures.append('IRQ not strongly defined: '+name)
        text,data,bss=map(int,sizes.splitlines()[1].split()[:3])
        if text+data>256*1024 or data+bss>64*1024:failures.append('Memory exceeds F401CC capacity')
        results.append({'preset':preset,'failures':failures,'size':sizes,'program_headers':ph,'text':text,'data':data,'bss':bss})
        print(preset, 'FAIL '+str(failures) if failures else 'PASS',text,data,bss)
    (ROOT/'test-results/artifacts.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
    return int(any(r['failures'] for r in results))
if __name__=='__main__':sys.exit(main())
