//#include ".\PSINSCore\kfapp.h"
#include "kfapp.h"

#define FRQ				200
#define TS				(1.0/FRQ)

//#define PathIn			"D:\\ygm2020\\高精度定位定向系统后处理软件V1.2\\PSINS-POS\\Data\\"
//#define PathOut			"D:\\ygm2020\\高精度定位定向系统后处理软件V1.2\\PSINS-POS\\Data\\"
//#define FileIn			"imugps.bin"

#define PathIn			"D:\\Datum\\20_11_28\\"
#define PathOut			"D:\\Datum\\20_11_28\\"
//#define FileIn			"imugnss.bin"
#define FileIn			"imugnss2.txt"
//#define FileIn			"data.bin"

int filetype = 0;


#define FileOut1		"posres.bin"
#define FileOut2		"fusion.bin"
//#define SkipT			20
//#define AlignT			100
//#define NavigationT		2400

#define SkipT			1		//刚开始跳过的时间
#define AlignT			100		//初始对准的时间
#define NavigationT		5000	//导航使用的时间

// Data input format (15-double-row X n-column):
// 0-2 Gyro_X/Y/Z(rad), 3-5 Acc_X/Y/Z(m/s), 6 IMU_t(sec)
// 7-9 GPS_VE/N/U(m/s), 10-12 GPS_Lat/Lon/Hgt(rad/rad/m), 13 GPS_Valid(0/1), 14 dt=IMU_t-GPS_t(sec)

int main(void)
{

	string str(FileIn);
	if (str.substr(str.size()-3,3)=="bin")
	{
		filetype = -15;
	}
	else if(str.substr(str.size()-3,3)=="txt")
	{
		fstream txtinfile;
		filetype = 15;
		while (getline(txtinfile, strs));
	}

	assert(filetype);

	CFileRdWt::Dir(PathIn, PathOut);
	CFileRdWt fimu(FileIn, filetype), fres(FileOut1);
	CVect3 pos0;
	ImuGpsData *pImuGps = (ImuGpsData*)&fimu.buff[0];

	// skip rows
	fimu.load(SkipT*FRQ);
	while(pImuGps->posgps.i<0.1) fimu.load(1);	pos0 = pImuGps->posgps;
	printf("%lf,%lf,%lf,%lf,%lf,%lf\n",pImuGps->wm.i, pImuGps->wm.j, pImuGps->wm.k,pImuGps->vm.i, pImuGps->vm.j, pImuGps->vm.k);
	printf("%lf,%lf,%lf,%lf,%lf,%lf\n", pImuGps->posgps.i, pImuGps->posgps.j, pImuGps->posgps.k, pImuGps->vngps.i, pImuGps->vngps.j, pImuGps->vngps.k);
	printf("%lf,%lf,%lf\n",pImuGps->t, pImuGps->gpsValid, pImuGps->dt);

	// initial alignment		初始对准
	printf("Initial alignment...\n");
	CAligni0 aln(pImuGps->posgps);
	for(int ii=0; ii<AlignT*FRQ; ii++)	{ fimu.load(1);	aln.Update(&pImuGps->wm, &pImuGps->vm, 1, TS); }

	CKFApp kf(TS);
	kf.lvGPS = CVect3(1.0, 0.5, 0.5);
	kf.Init(CSINS(aln.qnb, O31, pos0, pImuGps->t),0);

	// data loading
	printf("Data loading...\n");
	int records = 0;
	if (filetype == 15)		//read bin file
	{
		records = fimu.filesize() / (15 * sizeof(double)) - (SkipT + AlignT) * FRQ;		//find remain size	剩下还有多少的行数
		records = min(records, NavigationT * FRQ);
		CRMemory memImu(records, 15 * sizeof(double));
		records = fimu.load(memImu.get(0), records * 15 * sizeof(double)) / (15 * sizeof(double));
		CRMemory memFusion(records, sizeof(FXPT));
	}
	else if (filetype == -15)
	{
		records = 

	}


	// forward filtering
	printf("Forward filtering...\n");
	int i;
	for(i=0; i<records; i++)
	{
		pImuGps = (ImuGpsData*)memImu.get(i);
		kf.Update(&pImuGps->wm, &pImuGps->vm, 1, TS);
		if(pImuGps->gpsValid>0.1 && (pImuGps->t<700 || pImuGps->t>1000))
		{
			kf.SetMeasGPS(pImuGps->posgps, pImuGps->vngps, pImuGps->dt);
		}
		if(i%20==0 || pImuGps->gpsValid>0.1)
		{
			fres <<kf.sins.att<<kf.vnRes<<kf.posRes<<kf.sins.eb<<kf.sins.db<<pImuGps->vngps<<pImuGps->posgps
				 <<kf<<pImuGps->t;
		}
		memFusion.push((BYTE*)&kf.xpt);
		if((i+1)%(10*FRQ)==0) printf("\r\t%d (%d%%)\t", (i+1)/FRQ, 100*(i+1)/records);
	}

	// backward filtering
	kf.Reverse();
	printf("Data length(s) = %.3f.\nBackward filtering...\n", (float)records/FRQ);
	for(i--; i>=0; i--)
	{
		pImuGps = (ImuGpsData*)memImu.get(i);
		pImuGps->wm = -pImuGps->wm;  pImuGps->vngps = -pImuGps->vngps;
		if(pImuGps->gpsValid>0.1 && (pImuGps->t<700 || pImuGps->t>1000))
		{
			kf.SetMeasGPS(pImuGps->posgps, pImuGps->vngps, -pImuGps->dt);
		}
		if(i%20==0 || pImuGps->gpsValid>0.1)
		{
			fres <<kf.sins.att<<kf.vnRes<<kf.posRes<<kf.sins.eb<<kf.sins.db<<pImuGps->vngps<<pImuGps->posgps
				 <<kf<<pImuGps->t;
		}
		FXPT *pxpt = (FXPT*)memFusion.get(i);  pxpt->t = pImuGps->t;
		kf.xpt.vn = -kf.xpt.vn;
		fusion(&pxpt->att.i, &pxpt->Patt.i, &kf.xpt.att.i, &kf.xpt.Patt.i, 9);
		kf.Update(&pImuGps->wm, &pImuGps->vm, 1, TS);
		if(i%(10*FRQ)==0) printf("\r\t%d (%d%%)\t", i/FRQ, 100-100*i/records);
	}
	CFileRdWt ffsn(FileOut2); 
	ffsn<<memFusion;
	printf("\nFusion finished!\n");

	return 0;
}
