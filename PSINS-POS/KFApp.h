/* KFApp c++ hearder file KFApp.h */
/*
	By     : Yan Gongmin @ NWPU
	Date   : 2020-03-22
	From   : College of Automation, 
	         Northwestern Polytechnical University, 
			 Xi'an 710072, China
*/

#ifndef _KFAPP_H
#define _KFAPP_H

#include "PSINS.h"

typedef struct {
	CVect3 wm, vm;
	double t;
	CVect3 vngps, posgps;
	double gpsValid, dt;
} ImuGpsData;

typedef struct {
	CVect3 att, vn, pos, Patt, Pvn, Ppos;
	double t;
} FXPT;

class CKFApp:public CSINSTDKF
{
public:
	CVect3 lvGPS, vnRes, posRes;
	FXPT xpt;

	CKFApp(double ts);
	virtual void Init(const CSINS &sins0, int grade=-1);
	virtual void SetMeas(void) {};
	void SetMeasGPS(const CVect3 &posGPS, const CVect3 &vnGPS=O31, double dt=0.0);
	int Update(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts);
	void Reverse(void);
};

#endif

