"""Bit-accurate (integer) model of wiener_cplx_top.v.  Complex 64-pt STFT, hop 16, Hann, 75% overlap,
ONE scalar noise level from the first full frame (largest bin and +-2 bins excluded), decision-directed
Wiener gain.  All arithmetic is integer exactly as in the RTL, except the FFT/IFFT, which the RTL does with
the Xilinx IP (modelled here as an exact FFT rounded to integers)."""
import numpy as np
N=64; HOP=16; NSKIP=3; EXCL=2
HANN=np.array([int(round((0.5-0.5*np.cos(2*np.pi*n/N))*32768)) for n in range(N)],dtype=np.int64)  # Q1.15, 0..32768
ALPHA=int(round(0.98*65536)); BETA=int(round(0.02*65536)); XI_MIN=17; GAM_MAX=(1<<36)-1
RANK=16; RANK_K=56951; NITER=2; BIAS_K=17707; OLA_K=21845            # 2/3 in Q1.15  (1/sum(hann^2) = 1/1.5)
def sat(v,bits):
    lo=-(1<<(bits-1)); hi=(1<<(bits-1))-1; return max(lo,min(hi,v))
def rshift_round(v,s): return (v+(1<<(s-1)))>>s          # arithmetic, round half up
def fft_int(zr,zi,inverse=False):
    z=np.array(zr,dtype=np.float64)+1j*np.array(zi,dtype=np.float64)
    o=np.fft.ifft(z)*N if inverse else np.fft.fft(z)       # IP is UNSCALED both ways
    return np.rint(o.real).astype(np.int64),np.rint(o.imag).astype(np.int64)
def wiener_fixed(yr24,yi24):
    """yr24, yi24: int arrays, Q2.22 (beamformer m_data fields). Returns (outr24,outi24) Q2.22 ints, same length, latency removed."""
    n=len(yr24); nhop=(n+HOP-1)//HOP+ (N-HOP)//HOP
    xr=np.zeros(nhop*HOP,dtype=np.int64); xi=np.zeros(nhop*HOP,dtype=np.int64)
    xr[:n]=[rshift_round(int(v),9) for v in yr24]; xi[:n]=[rshift_round(int(v),9) for v in yi24]   # Q2.22 -> Q5.13
    ola_r=np.zeros(N,dtype=np.int64); ola_i=np.zeros(N,dtype=np.int64); qst=np.zeros(N,dtype=object); qst[:]=0
    noise_done=False; L=1; wr=0; out_r=[]; out_i=[]; fcnt=0
    buf_r=np.zeros(N,dtype=np.int64); buf_i=np.zeros(N,dtype=np.int64)
    for h in range(nhop):
        for i in range(HOP):
            buf_r[(wr+N-HOP+i)&(N-1)]=xr[h*HOP+i]; buf_i[(wr+N-HOP+i)&(N-1)]=xi[h*HOP+i]
        if fcnt>=NSKIP:
            wr_=[rshift_round(int(buf_r[(wr+k)&(N-1)])*int(HANN[k]),15) for k in range(N)]
            wi_=[rshift_round(int(buf_i[(wr+k)&(N-1)])*int(HANN[k]),15) for k in range(N)]
            Yr,Yi=fft_int(wr_,wi_)
            P=[int(Yr[k])**2+int(Yi[k])**2 for k in range(N)]
            if not noise_done:
                r16=sorted(P)[RANK]                              # rank-16 value (25th percentile); RTL: rank counting
                L=max((r16*RANK_K)>>14,1)                        # 25th pct of exponential = 0.2877*mean  -> x3.476
                for _ in range(NITER):                           # trimmed-mean refinement, drops signal bins
                    thr=4*L; s_=0; cnt=0
                    for k in range(N):
                        if P[k]<=thr: s_+=P[k]; cnt+=1
                    L=max(((s_*BIAS_K)>>14)//cnt,1)              # x1.0807 corrects truncation at 4*mean
                noise_done=True
            Zr=[0]*N; Zi=[0]*N
            for k in range(N):
                gam=min((P[k]<<16)//L,GAM_MAX)
                t1=rshift_round(int(qst[k])*ALPHA,16)
                gm1=gam-65536 if gam>65536 else 0
                t2=(gm1*BETA)>>8
                xi_=max(t1+t2,XI_MIN)
                G=(xi_<<24)//(xi_+(1<<24))
                g2=(G*G)>>24
                qst[k]=(g2*gam)>>16
                Zr[k]=sat(rshift_round(int(Yr[k])*G,30),18); Zi[k]=sat(rshift_round(int(Yi[k])*G,30),18)
            zr,zi=fft_int(Zr,Zi,inverse=True)
            for k in range(N):
                idx=(wr+k)&(N-1)
                ola_r[idx]+=rshift_round(int(zr[k])*int(HANN[k]),15); ola_i[idx]+=rshift_round(int(zi[k])*int(HANN[k]),15)
        for i in range(HOP):
            idx=(wr+i)&(N-1)
            vr=rshift_round(int(ola_r[idx])*OLA_K,15); vi=rshift_round(int(ola_i[idx])*OLA_K,15)
            out_r.append(sat(vr<<9,24)); out_i.append(sat(vi<<9,24)); ola_r[idx]=0; ola_i[idx]=0
        wr=(wr+HOP)&(N-1); fcnt=min(fcnt+1,NSKIP)
    lat=N-HOP
    return np.array(out_r[lat:lat+n]),np.array(out_i[lat:lat+n])
