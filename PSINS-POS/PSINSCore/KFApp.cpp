#include "KFApp.h"

/***************************  class CKFApp  *********************************/
CKFApp::CKFApp(double ts):CSINSTDKF(34, 6)
{
	lvGPS = O31;
}

void CKFApp::Init(const CSINS &sins0, int grade)
{
	CSINSKF::Init(sins0);
	sins.lever(-lvGPS);
	sins.pos = sins.posL;
	Pk.SetDiag2(0.1*DEG, 0.1*DEG, 1.0*DEG,	 1.0, 1.0, 1.0,  10.0/RE, 10.0/RE, 10.0,
		0.01*DPH, 0.01*DPH, 0.01*DPH,	100.0*UG, 100.0*UG, 500.0*UG,
		1.0, 1.0, 1.0, 0.001,
		1000.0*PPM, 10.0*SEC, 10.0*SEC,  10.0*SEC, 1000.0*PPM, 10.0*SEC,  10.0*SEC, 10.0*SEC, 1000.0*PPM,
		1000.0*PPM, 10.0*SEC, 10.0*SEC,  1000.0*PPM, 10.0*SEC,  10.0*PPM
		);
	Pmin.Set2(1.1*SEC, 1.1*SEC, 10.0*SEC,	 0.001, 0.001, 0.001,  0.001/RE, 0.001/RE, 0.001,
		0.001*DPH, 0.001*DPH, 0.001*DPH,	10.0*UG, 10.0*UG, 20.0*UG,
		0.01, 0.01, 0.01,  0.0001,
		0.0*PPM, 0.0*SEC, 0.0*SEC,  0.0*SEC, 0.0*PPM, 0.0*SEC,  0.0*SEC, 0.0*SEC, 0.0*PPM,
		0.0*PPM, 0.0*SEC, 0.0*SEC,  0.0*PPM, 0.0*SEC,  0.0*PPM
		);
	Qt.Set2(0.01*glv.dpsh, 0.01*glv.dpsh, 0.01*glv.dpsh,  10.0*glv.ugpsHz, 10.0*glv.ugpsHz, 20.0*glv.ugpsHz,  0.0, 0.0, 0.0, 
		0.0, 0.0, 0.0,  0.0, 0.0, 0.0,
		0.0, 0.0, 0.0,  0.0,
		0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,
		0.0, 0.0, 0.0,  0.0, 0.0, 0.0
		);
	FBTau.Set(1.0, 1.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,
		1.0, 1.0, 1.0,  1.0, 1.0, 1.0,
		INF, INF, INF,  INF,
		INF, INF, INF,  INF, INF, INF,  INF, INF, INF,
		INF, INF, INF,  INF, INF, INF
		);
	Rt.Set2(0.10, 0.10, 0.10, .10/RE, .10/RE, .10);
	Rmax = Rt * 100.;  Rmin = Rt*0.01;  Rb = 0.9;
	rts = 1.0;
}
 
void CKFApp::SetMeasGPS(const CVect3 &posGPS, const CVect3 &vnGPS, double dt)
{
	if (!IsZero(posGPS) && sins.wnb.k<30.0 * DPS && sins.wnb.k>-30.0 * DPS)
	{	
		sins.lever(lvGPS);
		if (!IsZero(vnGPS))
		{
			*(CVect3*)&Zk.dd[0] = sins.vnL-sins.an*dt - vnGPS;
			SetMeasFlag(00007);
		}
		*(CVect3*)&Zk.dd[3] = sins.posL-sins.Mpv*sins.vn*dt - posGPS;
		SetMeasFlag(00070);
	}
}

int CKFApp::Update(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts)
{
	int res = TDUpdate(pwm, pvm, nSamples, ts, 5);
	CVect3 lv = (lvGPS+*(CVect3*)&Xk.dd[15]);
	vnRes = sins.vn + sins.Cnb*askew(sins.web)*lv + sins.an*Xk.dd[18];
	posRes = sins.pos + sins.MpvCnb*lv + sins.Mpv*sins.vn*Xk.dd[18];
	xpt.att = sins.att; xpt.vn = vnRes; xpt.pos = posRes;
	double *p=&xpt.Patt.i, *p1=&Pk.dd[0];
	for(int i=0; i<9; i++,p++,p1+=nq+1) *p = *p1;
	return res;
}

void CKFApp::Reverse(void)
{
	sins.eth.wie = -sins.eth.wie;  sins.vn = -sins.vn; sins.eb = -sins.eb;
	vnRes = -vnRes; xpt.vn = -xpt.vn;
	int idx[] = {3,4,5, 9,10,11, 18};
	for(int k=0; k<(sizeof(idx)/sizeof(int)); k++) Xk.dd[idx[k]] = -Xk.dd[idx[k]];
	TDReset();
	int idxp[] = {0,1, 3,4,5, 6,7,8, 14};
	Pset = diag(Pk);
	for(int p=0; p<(sizeof(idxp)/sizeof(int)); p++) Pset.dd[idxp[p]] = 10.0*Pset.dd[idxp[p]];
}
