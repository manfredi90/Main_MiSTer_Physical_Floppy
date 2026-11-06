# stxscan.py - list the anomalies (likely copy protection) in an STX file
import struct, sys
from collections import Counter
def scan(fn):
    d=open(fn,'rb').read(); p=16; rev=d[11]
    print(f"== {fn}: revision {rev}, {d[10]} track blocks")
    stats=Counter(); notes=[]
    for _ in range(d[10]):
        blk,fz,ns,fl,mfm,tn,rt=struct.unpack_from('<IIHHHBB',d,p)
        c,h=tn&0x7f,tn>>7
        stats[f'sectors/track={ns}']+=1
        stats[f'track length {mfm//10*10}-{mfm//10*10+9}']+=1
        if fz: notes.append(f"c{c:02d} h{h}: {fz} bytes of fuzzy mask")
        q=p+16
        for s in range(ns):
            off,bp,rtm=struct.unpack_from('<IHH',d,q+16*s); idb=d[q+16*s+8:q+16*s+14]; st=d[q+16*s+14]
            size=128<<(idb[3]&3)
            flags=[]
            if st&0x01: flags.append('timing table')
            if st&0x08: flags.append('ID CRC error' if st&0x10 else 'data CRC error')
            if st&0x10 and not st&0x08: flags.append('no data (RNF)')
            if st&0x20: flags.append('deleted data')
            if st&0x80: flags.append('fuzzy')
            if idb[0]!=c or idb[1]!=h: flags.append(f'ID track/side {idb[0]}/{idb[1]}')
            if idb[3]!=2: flags.append(f'size code {idb[3]} ({size} bytes)')
            if not 1<=idb[2]<=11: flags.append(f'sector number {idb[2]}')
            if rtm and abs(rtm-16384*size/512)/(16384*size/512)>0.03: flags.append(f'read time {rtm} us ({(rtm/(16384*size/512)-1)*100:+.1f}%)')
            if flags: notes.append(f"c{c:02d} h{h} sector #{s} id {idb[:4].hex(' ')}: "+', '.join(flags))
        p+=blk
    for k,v in sorted(stats.items()): print(f"  {k}: {v}")
    for n in notes: print("  "+n)
for fn in sys.argv[1:]: scan(fn)
