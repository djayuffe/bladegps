#define _CRT_SECURE_NO_DEPRECATE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <ctype.h>
#include <limits.h>

#include <time.h>
#ifdef _WIN32
#include "getopt.h"
#else
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#endif

#include "gpssim.h"
#ifdef BLADE_GPS
#include "bladegps.h"
#ifdef _WIN32
#include <conio.h>
#else
#include "getch.h"
#endif
#endif

int sinTable512[] = {
	   2,   5,   8,  11,  14,  17,  20,  23,  26,  29,  32,  35,  38,  41,  44,  47,
	  50,  53,  56,  59,  62,  65,  68,  71,  74,  77,  80,  83,  86,  89,  91,  94,
	  97, 100, 103, 105, 108, 111, 114, 116, 119, 122, 125, 127, 130, 132, 135, 138,
	 140, 143, 145, 148, 150, 153, 155, 157, 160, 162, 164, 167, 169, 171, 173, 176,
	 178, 180, 182, 184, 186, 188, 190, 192, 194, 196, 198, 200, 202, 204, 205, 207,
	 209, 210, 212, 214, 215, 217, 218, 220, 221, 223, 224, 225, 227, 228, 229, 230,
	 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 241, 242, 243, 244, 244, 245,
	 245, 246, 247, 247, 248, 248, 248, 249, 249, 249, 249, 250, 250, 250, 250, 250,
	 250, 250, 250, 250, 250, 249, 249, 249, 249, 248, 248, 248, 247, 247, 246, 245,
	 245, 244, 244, 243, 242, 241, 241, 240, 239, 238, 237, 236, 235, 234, 233, 232,
	 230, 229, 228, 227, 225, 224, 223, 221, 220, 218, 217, 215, 214, 212, 210, 209,
	 207, 205, 204, 202, 200, 198, 196, 194, 192, 190, 188, 186, 184, 182, 180, 178,
	 176, 173, 171, 169, 167, 164, 162, 160, 157, 155, 153, 150, 148, 145, 143, 140,
	 138, 135, 132, 130, 127, 125, 122, 119, 116, 114, 111, 108, 105, 103, 100,  97,
	  94,  91,  89,  86,  83,  80,  77,  74,  71,  68,  65,  62,  59,  56,  53,  50,
	  47,  44,  41,  38,  35,  32,  29,  26,  23,  20,  17,  14,  11,   8,   5,   2,
	  -2,  -5,  -8, -11, -14, -17, -20, -23, -26, -29, -32, -35, -38, -41, -44, -47,
	 -50, -53, -56, -59, -62, -65, -68, -71, -74, -77, -80, -83, -86, -89, -91, -94,
	 -97,-100,-103,-105,-108,-111,-114,-116,-119,-122,-125,-127,-130,-132,-135,-138,
	-140,-143,-145,-148,-150,-153,-155,-157,-160,-162,-164,-167,-169,-171,-173,-176,
	-178,-180,-182,-184,-186,-188,-190,-192,-194,-196,-198,-200,-202,-204,-205,-207,
	-209,-210,-212,-214,-215,-217,-218,-220,-221,-223,-224,-225,-227,-228,-229,-230,
	-232,-233,-234,-235,-236,-237,-238,-239,-240,-241,-241,-242,-243,-244,-244,-245,
	-245,-246,-247,-247,-248,-248,-248,-249,-249,-249,-249,-250,-250,-250,-250,-250,
	-250,-250,-250,-250,-250,-249,-249,-249,-249,-248,-248,-248,-247,-247,-246,-245,
	-245,-244,-244,-243,-242,-241,-241,-240,-239,-238,-237,-236,-235,-234,-233,-232,
	-230,-229,-228,-227,-225,-224,-223,-221,-220,-218,-217,-215,-214,-212,-210,-209,
	-207,-205,-204,-202,-200,-198,-196,-194,-192,-190,-188,-186,-184,-182,-180,-178,
	-176,-173,-171,-169,-167,-164,-162,-160,-157,-155,-153,-150,-148,-145,-143,-140,
	-138,-135,-132,-130,-127,-125,-122,-119,-116,-114,-111,-108,-105,-103,-100, -97,
	 -94, -91, -89, -86, -83, -80, -77, -74, -71, -68, -65, -62, -59, -56, -53, -50,
	 -47, -44, -41, -38, -35, -32, -29, -26, -23, -20, -17, -14, -11,  -8,  -5,  -2
};

int cosTable512[] = {
	 250, 250, 250, 250, 250, 249, 249, 249, 249, 248, 248, 248, 247, 247, 246, 245,
	 245, 244, 244, 243, 242, 241, 241, 240, 239, 238, 237, 236, 235, 234, 233, 232,
	 230, 229, 228, 227, 225, 224, 223, 221, 220, 218, 217, 215, 214, 212, 210, 209,
	 207, 205, 204, 202, 200, 198, 196, 194, 192, 190, 188, 186, 184, 182, 180, 178,
	 176, 173, 171, 169, 167, 164, 162, 160, 157, 155, 153, 150, 148, 145, 143, 140,
	 138, 135, 132, 130, 127, 125, 122, 119, 116, 114, 111, 108, 105, 103, 100,  97,
	  94,  91,  89,  86,  83,  80,  77,  74,  71,  68,  65,  62,  59,  56,  53,  50,
	  47,  44,  41,  38,  35,  32,  29,  26,  23,  20,  17,  14,  11,   8,   5,   2,
	  -2,  -5,  -8, -11, -14, -17, -20, -23, -26, -29, -32, -35, -38, -41, -44, -47,
	 -50, -53, -56, -59, -62, -65, -68, -71, -74, -77, -80, -83, -86, -89, -91, -94,
	 -97,-100,-103,-105,-108,-111,-114,-116,-119,-122,-125,-127,-130,-132,-135,-138,
	-140,-143,-145,-148,-150,-153,-155,-157,-160,-162,-164,-167,-169,-171,-173,-176,
	-178,-180,-182,-184,-186,-188,-190,-192,-194,-196,-198,-200,-202,-204,-205,-207,
	-209,-210,-212,-214,-215,-217,-218,-220,-221,-223,-224,-225,-227,-228,-229,-230,
	-232,-233,-234,-235,-236,-237,-238,-239,-240,-241,-241,-242,-243,-244,-244,-245,
	-245,-246,-247,-247,-248,-248,-248,-249,-249,-249,-249,-250,-250,-250,-250,-250,
	-250,-250,-250,-250,-250,-249,-249,-249,-249,-248,-248,-248,-247,-247,-246,-245,
	-245,-244,-244,-243,-242,-241,-241,-240,-239,-238,-237,-236,-235,-234,-233,-232,
	-230,-229,-228,-227,-225,-224,-223,-221,-220,-218,-217,-215,-214,-212,-210,-209,
	-207,-205,-204,-202,-200,-198,-196,-194,-192,-190,-188,-186,-184,-182,-180,-178,
	-176,-173,-171,-169,-167,-164,-162,-160,-157,-155,-153,-150,-148,-145,-143,-140,
	-138,-135,-132,-130,-127,-125,-122,-119,-116,-114,-111,-108,-105,-103,-100, -97,
	 -94, -91, -89, -86, -83, -80, -77, -74, -71, -68, -65, -62, -59, -56, -53, -50,
	 -47, -44, -41, -38, -35, -32, -29, -26, -23, -20, -17, -14, -11,  -8,  -5,  -2,
	   2,   5,   8,  11,  14,  17,  20,  23,  26,  29,  32,  35,  38,  41,  44,  47,
	  50,  53,  56,  59,  62,  65,  68,  71,  74,  77,  80,  83,  86,  89,  91,  94,
	  97, 100, 103, 105, 108, 111, 114, 116, 119, 122, 125, 127, 130, 132, 135, 138,
	 140, 143, 145, 148, 150, 153, 155, 157, 160, 162, 164, 167, 169, 171, 173, 176,
	 178, 180, 182, 184, 186, 188, 190, 192, 194, 196, 198, 200, 202, 204, 205, 207,
	 209, 210, 212, 214, 215, 217, 218, 220, 221, 223, 224, 225, 227, 228, 229, 230,
	 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 241, 242, 243, 244, 244, 245,
	 245, 246, 247, 247, 248, 248, 248, 249, 249, 249, 249, 250, 250, 250, 250, 250
};

// Receiver antenna attenuation in dB for boresight angle = 0:5:180 [deg]
double ant_pat_db[37] = {
	 0.00,  0.00,  0.22,  0.44,  0.67,  1.11,  1.56,  2.00,  2.44,  2.89,  3.56,  4.22,
	 4.89,  5.56,  6.22,  6.89,  7.56,  8.22,  8.89,  9.78, 10.67, 11.56, 12.44, 13.33,
	14.44, 15.56, 16.67, 17.78, 18.89, 20.00, 21.33, 22.67, 24.00, 25.56, 27.33, 29.33,
	31.56
};

int allocatedSat[MAX_SAT];

static unsigned long uraIndexFromAccuracy(double accuracy);

/*! \brief Subtract two vectors of double
 *  \param[out] y Result of subtraction
 *  \param[in] x1 Minuend of subtracion
 *  \param[in] x2 Subtrahend of subtracion
 */
void subVect(double *y, const double *x1, const double *x2)
{
	y[0] = x1[0]-x2[0];
	y[1] = x1[1]-x2[1];
	y[2] = x1[2]-x2[2];

	return;
}

/*! \brief Compute Norm of Vector
 *  \param[in] x Input vector
 *  \returns Length (Norm) of the input vector
 */
double normVect(const double *x)
{
	return(sqrt(x[0]*x[0]+x[1]*x[1]+x[2]*x[2]));
}

/*! \brief Compute dot-product of two vectors
 *  \param[in] x1 First multiplicand
 *  \param[in] x2 Second multiplicand
 *  \returns Dot-product of both multiplicands
 */
double dotProd(const double *x1, const double *x2)
{
	return(x1[0]*x2[0]+x1[1]*x2[1]+x1[2]*x2[2]);
}

/* !\brief generate the C/A code sequence for a given Satellite Vehicle PRN
 *  \param[in] prn PRN nuber of the Satellite Vehicle
 *  \param[out] ca Caller-allocated integer array of 1023 bytes
 */
void codegen(int *ca, int prn)
{
	int delay[] = {
		  5,   6,   7,   8,  17,  18, 139, 140, 141, 251,
		252, 254, 255, 256, 257, 258, 469, 470, 471, 472,
		473, 474, 509, 512, 513, 514, 515, 516, 859, 860,
		861, 862, 863, 950, 947, 948, 950};
	
	int g1[CA_SEQ_LEN], g2[CA_SEQ_LEN];
	int r1[N_DWRD_SBF], r2[N_DWRD_SBF];
	int c1, c2;
	int i,j;

	if (prn<1 || prn>MAX_SAT)
		return;

	for (i=0; i<N_DWRD_SBF; i++)
		r1[i] = r2[i] = -1;

	for (i=0; i<CA_SEQ_LEN; i++)
	{
		g1[i] = r1[9];
		g2[i] = r2[9];
		c1 = r1[2]*r1[9];
		c2 = r2[1]*r2[2]*r2[5]*r2[7]*r2[8]*r2[9];

		for (j=9; j>0; j--) 
		{
			r1[j] = r1[j-1];
			r2[j] = r2[j-1];
		}
		r1[0] = c1;
		r2[0] = c2;
	}

	for (i=0,j=CA_SEQ_LEN-delay[prn-1]; i<CA_SEQ_LEN; i++,j++)
		ca[i] = (1-g1[i]*g2[j%CA_SEQ_LEN])/2;
	
	return;
}

/*! \brief Convert a UTC date into a GPS date
 *  \param[in] t input date in UTC form
 *  \param[out] g output date in GPS form
 */
void date2gps(const datetime_t *t, gpstime_t *g)
{
	int doy[12] = {0,31,59,90,120,151,181,212,243,273,304,334};
	long years = (long)t->y - 1980L;
	long leap_days = ((t->y-1)/4 - (t->y-1)/100 + (t->y-1)/400)
		- (1979/4 - 1979/100 + 1979/400);
	long de;

	de = years*365L + leap_days + doy[t->m-1] + t->d - 6;
	if (t->m > 2 && ((t->y%4 == 0 && t->y%100 != 0) || t->y%400 == 0))
		de++;

	// Convert time to GPS weeks and seconds.
	g->week = (int)(de / 7L);
	g->sec = (double)(de%7L)*SECONDS_IN_DAY + t->hh*SECONDS_IN_HOUR
		+ t->mm*SECONDS_IN_MINUTE + t->sec;

	return;
}

void gps2date(const gpstime_t *g, datetime_t *t)
{
	long c,d,e,f;
	double gsec = round(g->sec);

	// Convert Julian day number to calendar date
	c = (long)(7.0*(double)g->week + floor(gsec/86400.0)+2444245.0) + 1537;
	d = (long)(((double)c-122.1)/365.25);
	e = 365*d + d/4;
	f = (long)((double)(c-e)/30.6001 );

	t->d = (int)(c - e - (long)(30.6001*(double)f));
	t->m = (int)(f - 1L - 12L*(f/14L));
	t->y = (int)(d - 4715L - ((7L + (long)t->m)/10L));

	t->hh = ((int)(gsec/3600.0))%24;
	t->mm = ((int)(gsec/60.0))%60;
	t->sec = gsec - 60.0*floor(gsec/60.0);

	return;
}

/*! \brief Convert Earth-centered Earth-fixed (ECEF) into Lat/Long/Heighth
 *  \param[in] xyz Input Array of X, Y and Z ECEF coordinates
 *  \param[out] llh Output Array of Latitude, Longitude and Height
 */
void xyz2llh(const double *xyz, double *llh)
{
	double a,eps,e,e2;
	double x,y,z;
	double rho2,dz,zdz,nh,slat,n,dz_new;

	a = WGS84_RADIUS;
	e = WGS84_ECCENTRICITY;

	eps = 1.0e-6;
	e2 = e*e;

	if (normVect(xyz)<eps)
	{
		// Invalid ECEF vector
		llh[0] = 0.0;
		llh[1] = 0.0;
		llh[2] = -a;

		return;
	}

	x = xyz[0];
	y = xyz[1];
	z = xyz[2];

	rho2 = x*x + y*y;
	dz = e2*z;

	while (1)
	{
		zdz = z + dz;
		nh = sqrt(rho2 + zdz*zdz);
		slat = zdz / nh;
		n = a / sqrt(1.0-e2*slat*slat);
		dz_new = n*e2*slat;

		if (fabs(dz-dz_new) < eps)
			break;

		dz = dz_new;
	}

	llh[0] = atan2(zdz, sqrt(rho2));
	llh[1] = atan2(y, x);
	llh[2] = nh - n;

	return;
}

/*! \brief Convert Lat/Long/Height into Earth-centered Earth-fixed (ECEF)
 *  \param[in] llh Input Array of Latitude, Longitude and Height
 *  \param[out] xyz Output Array of X, Y and Z ECEF coordinates
 */
void llh2xyz(const double *llh, double *xyz)
{
	double n;
	double a;
	double e;
	double e2;
	double clat;
	double slat;
	double clon;
	double slon;
	double d,nph;
	double tmp;

	a = WGS84_RADIUS;
	e = WGS84_ECCENTRICITY;
	e2 = e*e;

	clat = cos(llh[0]);
	slat = sin(llh[0]);
	clon = cos(llh[1]);
	slon = sin(llh[1]);
	d = e*slat;

	n = a/sqrt(1.0-d*d);
	nph = n + llh[2];

	tmp = nph*clat;
	xyz[0] = tmp*clon;
	xyz[1] = tmp*slon;
	xyz[2] = ((1.0-e2)*n + llh[2])*slat;

	return;
}

/*! \brief Compute the intermediate matrix for LLH to ECEF
 *  \param[in] llh Input position in Latitude-Longitude-Height format
 *  \param[out] t Three-by-Three output matrix
 */
void ltcmat(const double *llh, double t[3][3])
{
	double slat, clat;
	double slon, clon;

	slat = sin(llh[0]);
	clat = cos(llh[0]);
	slon = sin(llh[1]);
	clon = cos(llh[1]);

	t[0][0] = -slat*clon;
	t[0][1] = -slat*slon;
	t[0][2] = clat;
	t[1][0] = -slon;
	t[1][1] = clon;
	t[1][2] = 0.0;
	t[2][0] = clat*clon;
	t[2][1] = clat*slon;
	t[2][2] = slat;

	return;
}

/*! \brief Convert Earth-centered Earth-Fixed to ?
 *  \param[in] xyz Input position as vector in ECEF format
 *  \param[in] t Intermediate matrix computed by \ref ltcmat
 *  \param[out] neu Output position as North-East-Up format
 */
void ecef2neu(const double *xyz, double t[3][3], double *neu)
{
	neu[0] = t[0][0]*xyz[0] + t[0][1]*xyz[1] + t[0][2]*xyz[2];
	neu[1] = t[1][0]*xyz[0] + t[1][1]*xyz[1] + t[1][2]*xyz[2];
	neu[2] = t[2][0]*xyz[0] + t[2][1]*xyz[1] + t[2][2]*xyz[2];

	return;
}

/*! \brief Convert North-Eeast-Up to Azimuth + Elevation
 *  \param[in] neu Input position in North-East-Up format
 *  \param[out] azel Output array of azimuth + elevation as double
 */
void neu2azel(double *azel, const double *neu)
{
	double ne;

	azel[0] = atan2(neu[1],neu[0]);
	if (azel[0]<0.0)
		azel[0] += (2.0*PI);

	ne = sqrt(neu[0]*neu[0] + neu[1]*neu[1]);
	azel[1] = atan2(neu[2], ne);

	return;
}

/*! \brief Compute Satellite position, velocity and clock at given time
 *  \param[in] eph Ephemeris data of the satellite
 *  \param[in] g GPS time at which position is to be computed
 *  \param[out] pos Computed position (vector)
 *  \param[out] vel Computed velociy (vector)
 *  \param[clk] clk Computed clock
 */
void satpos(ephem_t eph, gpstime_t g, double *pos, double *vel, double *clk)
{
	// Computing Satellite Velocity using the Broadcast Ephemeris
	// http://www.ngs.noaa.gov/gps-toolbox/bc_velo.htm

	double tk;
	double mk;
	double ek;
	double ekold;
	double ekdot;
	double cek,sek;
	double pk;
	double pkdot;
	double c2pk,s2pk;
	double uk;
	double ukdot;
	double cuk,suk;
	double ok;
	double sok,cok;
	double ik;
	double ikdot;
	double sik,cik;
	double rk;
	double rkdot;
	double xpk,ypk;
	double xpkdot,ypkdot;

	double relativistic, OneMinusecosE, tmp;

	tk = subGpsTime(g, eph.toe);
	while (tk > SECONDS_IN_HALF_WEEK)
		tk -= SECONDS_IN_WEEK;
	while (tk < -SECONDS_IN_HALF_WEEK)
		tk += SECONDS_IN_WEEK;

	mk = eph.m0 + eph.n*tk;
	ek = mk;
	ekold = ek + 1.0;
  
	OneMinusecosE = 0; // Suppress the uninitialized warning.
	for (int iteration = 0; iteration < 20 && fabs(ek-ekold)>1.0E-14; iteration++)
	{
		ekold = ek;
		OneMinusecosE = 1.0-eph.ecc*cos(ekold);
		ek = ek + (mk-ekold+eph.ecc*sin(ekold))/OneMinusecosE;
	}

	sek = sin(ek);
	cek = cos(ek);

	ekdot = eph.n/OneMinusecosE;

	relativistic = -4.442807633E-10*eph.ecc*eph.sqrta*sek;

	pk = atan2(eph.sq1e2*sek,cek-eph.ecc) + eph.aop;
	pkdot = eph.sq1e2*ekdot/OneMinusecosE;

	s2pk = sin(2.0*pk);
	c2pk = cos(2.0*pk);

	uk = pk + eph.cus*s2pk + eph.cuc*c2pk;
	suk = sin(uk);
	cuk = cos(uk);
	ukdot = pkdot*(1.0 + 2.0*(eph.cus*c2pk - eph.cuc*s2pk));

	rk = eph.A*OneMinusecosE + eph.crc*c2pk + eph.crs*s2pk;
	rkdot = eph.A*eph.ecc*sek*ekdot + 2.0*pkdot*(eph.crs*c2pk - eph.crc*s2pk);

	ik = eph.inc0 + eph.idot*tk + eph.cic*c2pk + eph.cis*s2pk;
	sik = sin(ik);
	cik = cos(ik);
	ikdot = eph.idot + 2.0*pkdot*(eph.cis*c2pk - eph.cic*s2pk);

	xpk = rk*cuk;
	ypk = rk*suk;
	xpkdot = rkdot*cuk - ypk*ukdot;
	ypkdot = rkdot*suk + xpk*ukdot;

	ok = eph.omg0 + tk*eph.omgkdot - OMEGA_EARTH*eph.toe.sec;
	sok = sin(ok);
	cok = cos(ok);

	pos[0] = xpk*cok - ypk*cik*sok;
	pos[1] = xpk*sok + ypk*cik*cok;
	pos[2] = ypk*sik;

	tmp = ypkdot*cik - ypk*sik*ikdot;

	vel[0] = -eph.omgkdot*pos[1] + xpkdot*cok - tmp*sok;
	vel[1] = eph.omgkdot*pos[0] + xpkdot*sok + tmp*cok;
	vel[2] = ypk*cik*ikdot + ypkdot*sik;

	// Satellite clock correction
	tk = subGpsTime(g, eph.toc);
	while (tk > SECONDS_IN_HALF_WEEK)
		tk -= SECONDS_IN_WEEK;
	while (tk < -SECONDS_IN_HALF_WEEK)
		tk += SECONDS_IN_WEEK;

	clk[0] = eph.af0 + tk*(eph.af1 + tk*eph.af2) + relativistic - eph.tgd;  
	clk[1] = eph.af1 + 2.0*tk*eph.af2
		- 4.442807633E-10*eph.ecc*eph.sqrta*cek*ekdot;

	return;
}

/*! \brief Compute Subframe from Ephemeris
 *  \param[in] eph Ephemeris of given SV
 *  \param[out] sbf Array of five sub-frames, 10 long words each
 */
void eph2sbf(const ephem_t eph, uint32_t sbf[5][N_DWRD_SBF])
{
	uint32_t wn;
	uint32_t toe;
	uint32_t toc;
	uint32_t iode;
	uint32_t iodc;
	int32_t deltan;
	int32_t cuc;
	int32_t cus;
	int32_t cic;
	int32_t cis;
	int32_t crc;
	int32_t crs;
	uint32_t ecc;
	uint32_t sqrta;
	int32_t m0;
	int32_t omg0;
	int32_t inc0;
	int32_t aop;
	int32_t omgdot;
	int32_t idot;
	int32_t af0;
	int32_t af1;
	int32_t af2;
	int32_t tgd;

	uint32_t ura;
	uint32_t dataId = UINT32_C(1);
	uint32_t sbf4_page25_svId = UINT32_C(63);
	uint32_t sbf5_page25_svId = UINT32_C(51);

	uint32_t wna;
	uint32_t toa;

	wn = (uint32_t)(eph.toe.week%1024);
	toe = (uint32_t)(eph.toe.sec/16.0);
	toc = (uint32_t)(eph.toc.sec/16.0);
	iode = (uint32_t)eph.iode;
	iodc = (uint32_t)eph.iodc;
	deltan = (int32_t)(eph.deltan/POW2_M43/PI);
	cuc = (int32_t)(eph.cuc/POW2_M29);
	cus = (int32_t)(eph.cus/POW2_M29);
	cic = (int32_t)(eph.cic/POW2_M29);
	cis = (int32_t)(eph.cis/POW2_M29);
	crc = (int32_t)(eph.crc/POW2_M5);
	crs = (int32_t)(eph.crs/POW2_M5);
	ecc = (uint32_t)(eph.ecc/POW2_M33);
	sqrta = (uint32_t)(eph.sqrta/POW2_M19);
	m0 = (int32_t)(eph.m0/POW2_M31/PI);
	omg0 = (int32_t)(eph.omg0/POW2_M31/PI);
	inc0 = (int32_t)(eph.inc0/POW2_M31/PI);
	aop = (int32_t)(eph.aop/POW2_M31/PI);
	omgdot = (int32_t)(eph.omgdot/POW2_M43/PI);
	idot = (int32_t)(eph.idot/POW2_M43/PI);
	af0 = (int32_t)(eph.af0/POW2_M31);
	af1 = (int32_t)(eph.af1/POW2_M43);
	af2 = (int32_t)(eph.af2/POW2_M55);
	tgd = (int32_t)(eph.tgd/POW2_M31);

	wna = (uint32_t)(eph.toe.week%256);
	toa = (uint32_t)(eph.toe.sec/4096.0);
	ura = (uint32_t)uraIndexFromAccuracy(eph.sv_accuracy);

	// Subframe 1
	sbf[0][0] = 0x8B0000UL<<6;
	sbf[0][1] = 0x1UL<<8;
	sbf[0][2] = ((wn&UINT32_C(0x3FF))<<20) | (ura<<14) |
		(((uint32_t)eph.sv_health&UINT32_C(0x3F))<<8) |
		(((iodc>>8)&UINT32_C(0x3))<<6);
	sbf[0][3] = 0UL;
	sbf[0][4] = 0UL;
	sbf[0][5] = 0UL;
	sbf[0][6] = ((uint32_t)tgd&UINT32_C(0xFF))<<6;
	sbf[0][7] = ((iodc&UINT32_C(0xFF))<<22) | ((toc&UINT32_C(0xFFFF))<<6);
	sbf[0][8] = (((uint32_t)af2&UINT32_C(0xFF))<<22) |
		(((uint32_t)af1&UINT32_C(0xFFFF))<<6);
	sbf[0][9] = ((uint32_t)af0&UINT32_C(0x3FFFFF))<<8;

	// Subframe 2
	sbf[1][0] = 0x8B0000UL<<6;
	sbf[1][1] = 0x2UL<<8;
	sbf[1][2] = ((iode&UINT32_C(0xFF))<<22) | (((uint32_t)crs&UINT32_C(0xFFFF))<<6);
	sbf[1][3] = (((uint32_t)deltan&UINT32_C(0xFFFF))<<14) |
		((((uint32_t)m0>>24)&UINT32_C(0xFF))<<6);
	sbf[1][4] = ((uint32_t)m0&UINT32_C(0xFFFFFF))<<6;
	sbf[1][5] = (((uint32_t)cuc&UINT32_C(0xFFFF))<<14) |
		(((ecc>>24)&UINT32_C(0xFF))<<6);
	sbf[1][6] = (ecc&UINT32_C(0xFFFFFF))<<6;
	sbf[1][7] = (((uint32_t)cus&UINT32_C(0xFFFF))<<14) |
		(((sqrta>>24)&UINT32_C(0xFF))<<6);
	sbf[1][8] = (sqrta&UINT32_C(0xFFFFFF))<<6;
	sbf[1][9] = (toe&UINT32_C(0xFFFF))<<14;

	// Subframe 3
	sbf[2][0] = 0x8B0000UL<<6;
	sbf[2][1] = 0x3UL<<8;
	sbf[2][2] = (((uint32_t)cic&UINT32_C(0xFFFF))<<14) |
		((((uint32_t)omg0>>24)&UINT32_C(0xFF))<<6);
	sbf[2][3] = ((uint32_t)omg0&UINT32_C(0xFFFFFF))<<6;
	sbf[2][4] = (((uint32_t)cis&UINT32_C(0xFFFF))<<14) |
		((((uint32_t)inc0>>24)&UINT32_C(0xFF))<<6);
	sbf[2][5] = ((uint32_t)inc0&UINT32_C(0xFFFFFF))<<6;
	sbf[2][6] = (((uint32_t)crc&UINT32_C(0xFFFF))<<14) |
		((((uint32_t)aop>>24)&UINT32_C(0xFF))<<6);
	sbf[2][7] = ((uint32_t)aop&UINT32_C(0xFFFFFF))<<6;
	sbf[2][8] = ((uint32_t)omgdot&UINT32_C(0xFFFFFF))<<6;
	sbf[2][9] = ((iode&UINT32_C(0xFF))<<22) |
		(((uint32_t)idot&UINT32_C(0x3FFF))<<8);

	// Subframe 4, page 25
	sbf[3][0] = 0x8B0000UL<<6;
	sbf[3][1] = 0x4UL<<8;
	sbf[3][2] = (dataId<<28) | (sbf4_page25_svId<<22);
	sbf[3][3] = 0UL;
	sbf[3][4] = 0UL;
	sbf[3][5] = 0UL;
	sbf[3][6] = 0UL;
	sbf[3][7] = 0UL;
	sbf[3][8] = 0UL;
	sbf[3][9] = 0UL;

	// Subframe 5, page 25
	sbf[4][0] = 0x8B0000UL<<6;
	sbf[4][1] = 0x5UL<<8;
	sbf[4][2] = (dataId<<28) | (sbf5_page25_svId<<22) |
		((toa&UINT32_C(0xFF))<<14) | ((wna&UINT32_C(0xFF))<<6);
	sbf[4][3] = 0UL;
	sbf[4][4] = 0UL;
	sbf[4][5] = 0UL;
	sbf[4][6] = 0UL;
	sbf[4][7] = 0UL;
	sbf[4][8] = 0UL;
	sbf[4][9] = 0UL;

	return;
}

/*! \brief Count number of bits set to 1
 *  \param[in] v long word in whihc bits are counted
 *  \returns Count of bits set to 1
 */
static uint32_t countBits(uint32_t v)
{
	uint32_t c;
	const int S[] = {1, 2, 4, 8, 16};
	const uint32_t B[] = {
		UINT32_C(0x55555555), UINT32_C(0x33333333), UINT32_C(0x0F0F0F0F),
		UINT32_C(0x00FF00FF), UINT32_C(0x0000FFFF)};

	c = v;
	c = ((c >> S[0]) & B[0]) + (c & B[0]);
	c = ((c >> S[1]) & B[1]) + (c & B[1]);
	c = ((c >> S[2]) & B[2]) + (c & B[2]);
	c = ((c >> S[3]) & B[3]) + (c & B[3]);
	c = ((c >> S[4]) & B[4]) + (c & B[4]);

	return(c);
}

/*! \brief Compute the Checksum for one given word of a subframe
 *  \param[in] source The input data
 *  \param[in] nib Does this word contain non-information-bearing bits?
 *  \returns Computed Checksum
 */
uint32_t computeChecksum(uint32_t source, int nib)
{
	/*
	Bits 31 to 30 = 2 LSBs of the previous transmitted word, D29* and D30*
	Bits 29 to  6 = Source data bits, d1, d2, ..., d24
	Bits  5 to  0 = Empty parity bits
	*/ 

	/*
	Bits 31 to 30 = 2 LSBs of the previous transmitted word, D29* and D30*
	Bits 29 to  6 = Data bits transmitted by the SV, D1, D2, ..., D24
	Bits  5 to  0 = Computed parity bits, D25, D26, ..., D30
	*/ 

	/*
	                  1            2           3
	bit    12 3456 7890 1234 5678 9012 3456 7890
	---    -------------------------------------
	D25    11 1011 0001 1111 0011 0100 1000 0000
	D26    01 1101 1000 1111 1001 1010 0100 0000
	D27    10 1110 1100 0111 1100 1101 0000 0000
	D28    01 0111 0110 0011 1110 0110 1000 0000
	D29    10 1011 1011 0001 1111 0011 0100 0000
	D30    00 1011 0111 1010 1000 1001 1100 0000
	*/

	const uint32_t bmask[6] = {
		UINT32_C(0x3B1F3480), UINT32_C(0x1D8F9A40), UINT32_C(0x2EC7CD00),
		UINT32_C(0x1763E680), UINT32_C(0x2BB1F340), UINT32_C(0x0B7A89C0)};

	uint32_t D;
	uint32_t d = source & UINT32_C(0x3FFFFFC0);
	uint32_t D29 = (source>>31)&UINT32_C(1);
	uint32_t D30 = (source>>30)&UINT32_C(1);

	if (nib) // Non-information bearing bits for word 2 and 10
	{
		/*
		Solve bits 23 and 24 to presearve parity check
		with zeros in bits 29 and 30.
		*/

		if ((D30 + countBits(bmask[4] & d)) % 2)
			d ^= (UINT32_C(1)<<6);
		if ((D29 + countBits(bmask[5] & d)) % 2)
			d ^= (UINT32_C(1)<<7);
	}

	D = d;
	if (D30)
		D ^= UINT32_C(0x3FFFFFC0);

	D |= ((D29 + countBits(bmask[0] & d)) % 2) << 5;
	D |= ((D30 + countBits(bmask[1] & d)) % 2) << 4;
	D |= ((D29 + countBits(bmask[2] & d)) % 2) << 3;
	D |= ((D30 + countBits(bmask[3] & d)) % 2) << 2;
	D |= ((D30 + countBits(bmask[4] & d)) % 2) << 1;
	D |= ((D29 + countBits(bmask[5] & d)) % 2);
	
	D &= 0x3FFFFFFFUL;
	//D |= (source & 0xC0000000UL); // Add D29* and D30* from source data bits

	return(D);
}

/*! \brief Replace all 'E' exponential designators to 'D'
 *  \param str String in which all occurrences of 'E' are replaced with *  'D'
 *  \param len Length of input string in bytes
 *  \returns Number of characters replaced
 */
int replaceExpDesignator(char *str, int len)
{
	int i,n=0;

	for (i=0; i<len; i++)
	{
		if (str[i]=='D')
		{
			n++;
			str[i] = 'E';
		}
	}
	
	return(n);
}

double subGpsTime(gpstime_t g1, gpstime_t g0)
{
	double dt;

	dt = g1.sec - g0.sec;
	dt += (double)(g1.week - g0.week) * SECONDS_IN_WEEK;

	return(dt);
}

void normalizeGpsTime(gpstime_t *g)
{
	while (g->sec >= SECONDS_IN_WEEK) {
		g->sec -= SECONDS_IN_WEEK;
		g->week++;
	}

	while (g->sec < 0.0) {
		g->sec += SECONDS_IN_WEEK;
		g->week--;
	}
}

int selectEphemerides(ephem_t selected[MAX_SAT],
	const ephem_t source[][MAX_SAT], int count, gpstime_t time)
{
	int selected_count = 0;
	int sv;

	memset(selected, 0, sizeof(ephem_t) * MAX_SAT);
	if (count <= 0 || count > EPHEM_ARRAY_SIZE)
		return 0;

	for (sv = 0; sv < MAX_SAT; sv++) {
		double best_age = HUGE_VAL;
		int best = -1;
		int set;

		for (set = 0; set < count; set++) {
			double age;
			double fit_hours;

			if (source[set][sv].vflg != 1 || source[set][sv].sv_health != 0)
				continue;

			age = subGpsTime(time, source[set][sv].toe);
			/* Do not pull an ephemeris arbitrarily from the future.  Thirty
			 * seconds allows for a rounded message boundary in archived files. */
			if (age < -30.0)
				continue;
			age = fabs(age);
			fit_hours = source[set][sv].fit_interval > 0.0 ?
				source[set][sv].fit_interval : DEFAULT_EPHEMERIS_FIT_HOURS;
			if (age > fit_hours * SECONDS_IN_HOUR / 2.0)
				continue;
			if (age < best_age) {
				best_age = age;
				best = set;
			}
		}

		if (best >= 0) {
			selected[sv] = source[best][sv];
			selected_count++;
		}
	}

	return selected_count;
}

static unsigned long uraIndexFromAccuracy(double accuracy)
{
	static const double ura_threshold[] = {
		2.4, 3.4, 4.85, 6.85, 9.65, 13.65, 24.0, 48.0,
		96.0, 192.0, 384.0, 768.0, 1536.0, 3072.0, 6144.0
	};
	unsigned long i;

	if (!isfinite(accuracy) || accuracy < 0.0)
		return 15UL;

	for (i = 0; i < sizeof(ura_threshold) / sizeof(ura_threshold[0]); i++)
		if (accuracy <= ura_threshold[i])
			return i;

	return 15UL;
}

static int rinex_line_has_fields(const char *str)
{
	return strlen(str) >= 79;
}

static int has_suffix(const char *value, const char *suffix)
{
	size_t value_length = strlen(value);
	size_t suffix_length = strlen(suffix);

	return value_length >= suffix_length &&
		strcmp(value + value_length - suffix_length, suffix) == 0;
}

#ifndef _WIN32
static FILE *open_rinex_stream(const char *fname, pid_t *decompressor_pid)
{
	int descriptors[2];
	pid_t pid;
	FILE *stream;

	*decompressor_pid = -1;
	if (!has_suffix(fname, ".gz") && !has_suffix(fname, ".Z"))
		return fopen(fname, "rt");

	if (pipe(descriptors) != 0)
		return NULL;

	pid = fork();
	if (pid < 0) {
		close(descriptors[0]);
		close(descriptors[1]);
		return NULL;
	}
	if (pid == 0) {
		close(descriptors[0]);
		if (dup2(descriptors[1], STDOUT_FILENO) < 0)
			_exit(127);
		close(descriptors[1]);
		execlp("gzip", "gzip", "-cd", "--", fname, (char *)NULL);
		_exit(127);
	}

	close(descriptors[1]);
	stream = fdopen(descriptors[0], "r");
	if (stream == NULL) {
		close(descriptors[0]);
		waitpid(pid, NULL, 0);
		return NULL;
	}
	*decompressor_pid = pid;
	return stream;
}

static int close_rinex_stream(FILE *stream, pid_t decompressor_pid)
{
	int close_status = fclose(stream);
	int child_status = 0;

	if (decompressor_pid > 0) {
		while (waitpid(decompressor_pid, &child_status, 0) < 0) {
			if (errno != EINTR)
				return -1;
		}
		if (!WIFEXITED(child_status) || WEXITSTATUS(child_status) != 0)
			return -1;
	}

	return close_status == 0 ? 0 : -1;
}
#endif

static int ephemeris_parameters_valid(const ephem_t *eph)
{
	return eph->toe.week >= 0 && eph->toe.sec >= 0.0 && eph->toe.sec < SECONDS_IN_WEEK &&
		eph->sqrta > 0.0 && isfinite(eph->sqrta) &&
		eph->ecc >= 0.0 && eph->ecc < 1.0 && isfinite(eph->ecc) &&
		isfinite(eph->deltan) && isfinite(eph->m0) &&
		isfinite(eph->omg0) && isfinite(eph->inc0) &&
		isfinite(eph->aop) && isfinite(eph->omgdot) && isfinite(eph->idot) &&
		isfinite(eph->cuc) && isfinite(eph->cus) &&
		isfinite(eph->cic) && isfinite(eph->cis) &&
		isfinite(eph->crc) && isfinite(eph->crs) &&
		isfinite(eph->af0) && isfinite(eph->af1) && isfinite(eph->af2) &&
		isfinite(eph->tgd) && eph->sv_health >= 0;
}

/*! \brief Read Ephemersi data from the RINEX Navigation file */
/*  \param[out] eph Array of Output SV ephemeris data
 *  \param[in] fname File name of the RINEX file
 *  \returns Number of sets of ephemerides in the file
 */
int readRinexNavAll(ephem_t eph[][MAX_SAT], const char *fname)
{
	FILE *fp;
	int ieph;
	int header_complete = 0;
	double rinex_version = 0.0;
	
	int sv;
	char str[MAX_CHAR];
	char tmp[20];

	datetime_t t;
	gpstime_t g;
	gpstime_t g0;
	double dt;
#ifndef _WIN32
	pid_t decompressor_pid;
#endif

#ifdef _WIN32
	if (has_suffix(fname, ".gz") || has_suffix(fname, ".Z"))
		return -1;
	if (NULL==(fp=fopen(fname, "rt")))
		return(-1);
#else
	if (NULL==(fp=open_rinex_stream(fname, &decompressor_pid)))
		return(-1);
#endif

	memset(eph, 0, sizeof(ephem_t) * EPHEM_ARRAY_SIZE * MAX_SAT);

	// Read and validate the RINEX 2 navigation header.
	while (1)
	{
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;
		if (rinex_version == 0.0)
			rinex_version = atof(str);

		if (strlen(str) >= 73 && strncmp(str+60, "END OF HEADER", 13)==0) {
			header_complete = 1;
			break;
		}
	}
	if (!header_complete || rinex_version < 2.0 || rinex_version >= 3.0) {
#ifdef _WIN32
		fclose(fp);
#else
		close_rinex_stream(fp, decompressor_pid);
#endif
		return -1;
	}

	g0.week = -1;
	ieph = 0;

	while (1)
	{
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;
		if (!rinex_line_has_fields(str))
			break;

		// PRN
		strncpy(tmp, str, 2);
		tmp[2] = 0;
		sv = atoi(tmp)-1;
		if (sv < 0 || sv >= MAX_SAT)
			break;

		// EPOCH
		strncpy(tmp, str+3, 2);
		tmp[2] = 0;
		t.y = atoi(tmp);
		t.y += t.y < 80 ? 2000 : 1900;

		strncpy(tmp, str+6, 2);
		tmp[2] = 0;
		t.m = atoi(tmp);

		strncpy(tmp, str+9, 2);
		tmp[2] = 0;
		t.d = atoi(tmp);

		strncpy(tmp, str+12, 2);
		tmp[2] = 0;
		t.hh = atoi(tmp);

		strncpy(tmp, str+15, 2);
		tmp[2] = 0;
		t.mm = atoi(tmp);

		strncpy(tmp, str+18, 4);
		tmp[4] = 0;
		t.sec = atof(tmp);

		date2gps(&t, &g);
		
		if (g0.week==-1)
			g0 = g;

		// Check current time of clock
		dt = subGpsTime(g, g0);
		
		if (dt>SECONDS_IN_HOUR)
		{
			g0 = g;
			ieph++; // a new set of ephemerides

			if (ieph>=EPHEM_ARRAY_SIZE)
				break;
		}

		// Date and time
		eph[ieph][sv].t = t;

		// SV CLK
		eph[ieph][sv].toc = g;

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19); // tmp[15]='E';
		eph[ieph][sv].af0 = atof(tmp);

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].af1 = atof(tmp);

		strncpy(tmp, str+60, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].af2 = atof(tmp);

		// BROADCAST ORBIT - 1
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;
		if (!rinex_line_has_fields(str))
			break;

		strncpy(tmp, str+3, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].iode = (int)atof(tmp);

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].crs = atof(tmp);

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].deltan = atof(tmp);

		strncpy(tmp, str+60, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].m0 = atof(tmp);

		// BROADCAST ORBIT - 2
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;
		if (!rinex_line_has_fields(str))
			break;

		strncpy(tmp, str+3, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].cuc = atof(tmp);

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].ecc = atof(tmp);

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].cus = atof(tmp);

		strncpy(tmp, str+60, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].sqrta = atof(tmp);

		// BROADCAST ORBIT - 3
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;
		if (!rinex_line_has_fields(str))
			break;

		strncpy(tmp, str+3, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].toe.sec = atof(tmp);

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].cic = atof(tmp);

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].omg0 = atof(tmp);

		strncpy(tmp, str+60, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].cis = atof(tmp);

		// BROADCAST ORBIT - 4
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;
		if (!rinex_line_has_fields(str))
			break;

		strncpy(tmp, str+3, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].inc0 = atof(tmp);

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].crc = atof(tmp);
		
		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].aop = atof(tmp);

		strncpy(tmp, str+60, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].omgdot = atof(tmp);

		// BROADCAST ORBIT - 5
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;
		if (!rinex_line_has_fields(str))
			break;

		strncpy(tmp, str+3, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].idot = atof(tmp);

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].toe.week = (int)atof(tmp);

		// BROADCAST ORBIT - 6
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;
		if (!rinex_line_has_fields(str))
			break;

		strncpy(tmp, str+3, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].sv_accuracy = atof(tmp);

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].sv_health = (int)atof(tmp);

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].tgd = atof(tmp);

		strncpy(tmp, str+60, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].iodc = (int)atof(tmp);

		// BROADCAST ORBIT - 7
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;
		eph[ieph][sv].fit_interval = DEFAULT_EPHEMERIS_FIT_HOURS;
		if (strlen(str) >= 41) {
			strncpy(tmp, str+22, 19);
			tmp[19] = 0;
			replaceExpDesignator(tmp, 19);
			if (atof(tmp) > 0.0)
				eph[ieph][sv].fit_interval = atof(tmp);
		}

		if (!ephemeris_parameters_valid(&eph[ieph][sv]))
			continue;

		// Set valid flag
		eph[ieph][sv].vflg = 1;

		// Update the working variables
		eph[ieph][sv].A = eph[ieph][sv].sqrta * eph[ieph][sv].sqrta;
		eph[ieph][sv].n = sqrt(GM_EARTH/(eph[ieph][sv].A*eph[ieph][sv].A*eph[ieph][sv].A)) + eph[ieph][sv].deltan;
		eph[ieph][sv].sq1e2 = sqrt(1.0 - eph[ieph][sv].ecc*eph[ieph][sv].ecc);
		eph[ieph][sv].omgkdot = eph[ieph][sv].omgdot - OMEGA_EARTH;
	}

#ifdef _WIN32
	fclose(fp);
#else
	if (close_rinex_stream(fp, decompressor_pid) != 0)
		return -1;
#endif
	
	if (g0.week>=0)
		ieph += 1; // Number of sets of ephemerides

	return(ieph);
}

/*! \brief Compute range between a satellite and the receiver
 *  \param[out] rho The computed range
 *  \param[in] eph Ephemeris data of the satellite
 *  \param[in] g GPS time at time of receiving the signal
 *  \param[in] xyz position of the receiver
 */
void computeRange(range_t *rho, ephem_t eph, gpstime_t g, double xyz[])
{
	double pos[3],vel[3],clk[2];
	double los[3];
	double tau = 0.07;
	double range,rate;
	double xrot,yrot;
	double rotation;
	double cosine,sine;
	gpstime_t transmit_time;
	int iteration;

	double llh[3],neu[3];
	double tmat[3][3];

	
	/* Solve transmit time iteratively. The satellite state is evaluated at
	 * transmission, then rotated into the receive-time ECEF frame (Sagnac
	 * correction). Two to four iterations are normally sufficient. */
	for (iteration = 0; iteration < 5; iteration++) {
		transmit_time = g;
		transmit_time.sec -= tau;
		normalizeGpsTime(&transmit_time);
		satpos(eph, transmit_time, pos, vel, clk);

		rotation = OMEGA_EARTH * tau;
		cosine = cos(rotation);
		sine = sin(rotation);
		xrot = cosine*pos[0] + sine*pos[1];
		yrot = -sine*pos[0] + cosine*pos[1];
		pos[0] = xrot;
		pos[1] = yrot;

		subVect(los, pos, xyz);
		range = normVect(los);
		if (fabs(range/SPEED_OF_LIGHT - tau) < 1.0e-12)
			break;
		tau = range/SPEED_OF_LIGHT;
	}

	// New observer to satellite vector and satellite range.
	subVect(los, pos, xyz);
	range = normVect(los);
	rho->d = range;

	// Pseudorange.
	rho->range = range - SPEED_OF_LIGHT*clk[0];

	/* Rotate velocity consistently with position. The small derivative of the
	 * Sagnac rotation is below the simulator's 100 ms Doppler update scale. */
	xrot = cosine*vel[0] + sine*vel[1];
	yrot = -sine*vel[0] + cosine*vel[1];
	vel[0] = xrot;
	vel[1] = yrot;

	// Relative velocity of SV and receiver.
	rate = dotProd(vel, los)/range;

	// Pseudorange rate.
	rho->rate = rate - SPEED_OF_LIGHT*clk[1];

	// Time of application.
	rho->g = g;

	// Azimuth and elevation angles.
	xyz2llh(xyz, llh);
	ltcmat(llh, tmat);
	ecef2neu(los, tmat, neu);
	neu2azel(rho->azel, neu);

	return;
}

/*! \brief Compute the code phase for a given channel (satellite)
 *  \param chan Channel on which we operate (is updated)
 *  \param[in] rho1 Current range, after \a dt has expired
 *  \param[in dt delta-t (time difference) in seconds
 */
void computeCodePhase(channel_t *chan, range_t rho1, double dt)
{
	double ms;
	int ims;
	double rhorate;
	
	if (dt <= 0.0)
		return;

	// Pseudorange rate, including receiver motion over the update interval.
	rhorate = (rho1.range - chan->rho0.range)/dt;

	// Carrier and code frequency.
	chan->f_carr = -rhorate/LAMBDA_L1;
	chan->f_code = CODE_FREQ + chan->f_carr*CARR_TO_CODE;

	// Initial code phase and data bit counters.
	ms = ((subGpsTime(chan->rho0.g, chan->g0)+6.0) - chan->rho0.range/SPEED_OF_LIGHT)*1000.0;
	ms = fmod(ms, (double)N_DWRD * 600.0);
	if (ms < 0.0)
		ms += (double)N_DWRD * 600.0;

	ims = (int)ms;
	chan->code_phase = (ms-(double)ims)*CA_SEQ_LEN; // in chip

	chan->iword = ims/600; // 1 word = 30 bits = 600 ms
	ims -= chan->iword*600;
			
	chan->ibit = ims/20; // 1 bit = 20 code = 20 ms
	ims -= chan->ibit*20;

	chan->icode = ims; // 1 code = 1 ms

	chan->codeCA = chan->ca[(int)chan->code_phase]*2-1;
	chan->dataBit = (int)((chan->dwrd[chan->iword]>>(29-chan->ibit)) & 0x1UL)*2-1;

	// Save current pseudorange
	chan->rho0 = rho1;

	return;
}

/*! \brief Read the list of user motions from the input file
 *  \param[out] xyz Output array of ECEF vectors for user motion
 *  \param[[in] filename File name of the text input file
 *  \returns Number of user data motion records read, -1 on error
 */

static int append_motion_segment(double **xyz,int *count,double output_time,
	double previous_time,const double previous[3],double current_time,
	const double current[3])
{
	double alpha;
	if(current_time<=previous_time||!isfinite(output_time))return -1;
	while(*count<USER_MOTION_SIZE&&output_time<=current_time+1.0e-9) {
		int axis;
		alpha=(output_time-previous_time)/(current_time-previous_time);
		if(alpha<0.0)alpha=0.0;if(alpha>1.0)alpha=1.0;
		for(axis=0;axis<3;axis++)xyz[*count][axis]=previous[axis]+alpha*(current[axis]-previous[axis]);
		(*count)++;output_time+=0.1;
	}
	return 0;
}

static int parse_motion_csv4(const char *line,double values[4])
{
	const char *cursor=line;
	char *end;
	unsigned int field;
	if(line==NULL||values==NULL)return -1;
	for(field=0U;field<4U;field++) {
		errno=0;values[field]=strtod(cursor,&end);
		if(errno!=0||end==cursor||!isfinite(values[field]))return -1;
		while(*end==' '||*end=='\t')end++;
		if(field<3U) {
			if(*end!=',')return -1;
			cursor=end+1;
		} else {
			while(isspace((unsigned char)*end))end++;
			if(*end!='\0')return -1;
		}
	}
	return 0;
}

//int readUserMotion(double xyz[USER_MOTION_SIZE][3], const char *filename)
int readUserMotion(double **xyz, const char *filename)
{
	FILE *fp;
	int numd=0,have_previous=0;
	char str[MAX_CHAR];
	double t,x,y,z,start_time=0.0,previous_time=0.0,previous[3]={0};

	if (NULL==(fp=fopen(filename,"rt")))
		return(-1);

	while(numd<USER_MOTION_SIZE)
	{
		double fields[4];
		if (fgets(str, MAX_CHAR, fp)==NULL)
			break;

		if ((strchr(str,'\n')==NULL&&!feof(fp))||parse_motion_csv4(str,fields)!=0) {
			fclose(fp);
			return -2;
		}
		t=fields[0];x=fields[1];y=fields[2];z=fields[3];

		if(!have_previous) {
			start_time=previous_time=t;previous[0]=xyz[0][0]=x;
			previous[1]=xyz[0][1]=y;previous[2]=xyz[0][2]=z;
			numd=1;have_previous=1;continue;
		}
		{
			double current[3]={x,y,z};
			if(append_motion_segment(xyz,&numd,start_time+0.1*(double)numd,
				previous_time,previous,t,current)!=0){fclose(fp);return -2;}
			previous_time=t;memcpy(previous,current,sizeof(previous));
		}
	}

	fclose(fp);

	return (numd);
}

int readLlhMotion(double **xyz, const char *filename)
{
	FILE *fp;
	int numd=0,have_previous=0;
	char str[MAX_CHAR];
	double time_seconds,latitude,longitude,height;
	double llh[3],current[3],previous[3]={0},start_time=0.0,previous_time=0.0;

	if (NULL==(fp=fopen(filename,"rt")))
		return -1;

	while(numd<USER_MOTION_SIZE) {
		double fields[4];
		if (fgets(str, MAX_CHAR, fp)==NULL)
			break;
		if ((strchr(str,'\n')==NULL&&!feof(fp))||parse_motion_csv4(str,fields)!=0) {
			fclose(fp);return -2;
		}
		time_seconds=fields[0];latitude=fields[1];longitude=fields[2];height=fields[3];
		if (latitude < -90.0 || latitude > 90.0 ||
			longitude < -180.0 || longitude > 180.0) {
			fclose(fp);
			return -2;
		}
		llh[0] = latitude/R2D;
		llh[1] = longitude/R2D;
		llh[2] = height;
		llh2xyz(llh,current);
		if(!have_previous){start_time=previous_time=time_seconds;
			memcpy(previous,current,sizeof(previous));memcpy(xyz[0],current,sizeof(current));
			numd=1;have_previous=1;continue;}
		if(append_motion_segment(xyz,&numd,start_time+0.1*(double)numd,
			previous_time,previous,time_seconds,current)!=0){fclose(fp);return -2;}
		previous_time=time_seconds;memcpy(previous,current,sizeof(previous));
	}

	fclose(fp);
	return numd;
}

static int nmea_checksum_valid(const char *sentence)
{
	const char *star;
	const char *tail;
	unsigned int expected,value=0U;
	if(sentence==NULL||sentence[0]!='$')return 0;
	star=strchr(sentence,'*');
	if(star==NULL)return 1;
	if(star[1]=='\0'||star[2]=='\0')return 0;
	if(!isxdigit((unsigned char)star[1])||!isxdigit((unsigned char)star[2]))return 0;
	if(sscanf(star+1,"%2x",&expected)!=1)return 0;
	for(tail=star+3;*tail!='\0';tail++)if(!isspace((unsigned char)*tail))return 0;
	for(sentence++;sentence<star;sentence++)value^=(unsigned char)*sentence;
	return value==expected;
}

static int nmea_time_seconds(const char *text,double *seconds)
{
	double raw,sec;
	int hour,minute;
	char *end;
	if(text==NULL||seconds==NULL)return -1;
	raw=strtod(text,&end);if(end==text||!isfinite(raw))return -1;
	while(isspace((unsigned char)*end))end++;
	if(*end!='\0')return -1;
	hour=(int)(raw/10000.0);minute=(int)(raw/100.0)%100;sec=fmod(raw,100.0);
	if(hour<0||hour>23||minute<0||minute>59||sec<0.0||sec>=61.0)return -1;
	*seconds=(double)hour*3600.0+(double)minute*60.0+sec;return 0;
}

static int nmea_decimal(const char *text,double *value)
{
	char *end;
	if(text==NULL||value==NULL||*text=='\0')return -1;
	*value=strtod(text,&end);
	if(end==text||!isfinite(*value))return -1;
	while(isspace((unsigned char)*end))end++;
	return *end=='\0'?0:-1;
}

static int nmea_coordinate(const char *text,const char *hemisphere,
	int maximum_degrees,double *radians)
{
	double raw,degrees,minutes,value;
	char positive,negative;
	if(nmea_decimal(text,&raw)!=0||raw<0.0||hemisphere==NULL||
		hemisphere[0]=='\0'||hemisphere[1]!='\0'||radians==NULL)return -1;
	positive=maximum_degrees==90?'N':'E';negative=maximum_degrees==90?'S':'W';
	if(hemisphere[0]!=positive&&hemisphere[0]!=negative)return -1;
	degrees=floor(raw/100.0);minutes=raw-degrees*100.0;
	if(degrees<0.0||degrees>(double)maximum_degrees||minutes<0.0||minutes>=60.0||
		(degrees==(double)maximum_degrees&&minutes>0.0))return -1;
	value=degrees+minutes/60.0;
	if(hemisphere[0]==negative)value=-value;
	*radians=value/R2D;return 0;
}

static int nmea_integer(const char *text,int *value)
{
	char *end;long parsed;
	if(text==NULL||value==NULL||*text=='\0')return -1;
	parsed=strtol(text,&end,10);
	while(isspace((unsigned char)*end))end++;
	if(end==text||*end!='\0'||parsed<INT_MIN||parsed>INT_MAX)return -1;
	*value=(int)parsed;return 0;
}

static int nmea_metre_unit(const char *text)
{
	if(text==NULL||*text!='M')return 0;
	text++;
	while(isspace((unsigned char)*text))text++;
	return *text=='\0';
}

//int readNmeaGGA(double xyz[USER_MOTION_SIZE][3], const char *filename)
int readNmeaGGA(double **xyz, const char *filename)
{
	FILE *fp;
	int numd = 0;
	char str[MAX_CHAR];
	char *token;
	double llh[3],pos[3];
	int fix_quality;
	double timestamp,start_time=0.0,previous_time=0.0,day_offset=0.0,previous[3]={0};
	int have_previous=0;

	if (NULL==(fp=fopen(filename,"rt")))
		return(-1);

	while (1)
	{
		if (fgets(str, MAX_CHAR, fp)==NULL)
			break;
		if(!nmea_checksum_valid(str))continue;
		{
			char *checksum=strchr(str,'*');
			if(checksum!=NULL)*checksum='\0';
		}

		token = strtok(str, ",");
		if (token == NULL || strlen(token) < 6)
			continue;

		if (strncmp(token+3, "GGA", 3)==0)
		{
			token=strtok(NULL, ",");
			if (token == NULL || nmea_time_seconds(token,&timestamp)!=0)
				continue;
			
			token = strtok(NULL, ","); // Latitude
			if (token == NULL)
				continue;
			{
				char *latitude=token;
				token = strtok(NULL, ","); // North or south
				if(nmea_coordinate(latitude,token,90,&llh[0])!=0)continue;
			}
			
			token = strtok(NULL, ","); // Longitude
			if (token == NULL)
				continue;
			{
				char *longitude=token;
				token = strtok(NULL, ","); // East or west
				if(nmea_coordinate(longitude,token,180,&llh[1])!=0)continue;
			}

			token = strtok(NULL, ","); // GPS fix
			if (nmea_integer(token,&fix_quality)!=0 || fix_quality <= 0)
				continue;
			token = strtok(NULL, ","); // Number of satellites
			if (token == NULL)
				continue;
			token = strtok(NULL, ","); // HDOP
			if (token == NULL)
				continue;

			token = strtok(NULL, ","); // Altitude above meas sea level
			if (nmea_decimal(token,&llh[2])!=0)
				continue;

			token = strtok(NULL, ","); // in meter
			if (!nmea_metre_unit(token))
				continue;

			token = strtok(NULL, ","); // Geoid height above WGS84 ellipsoid
			{
				double geoid;
				if(nmea_decimal(token,&geoid)!=0)continue;
				llh[2]+=geoid;
			}
			token=strtok(NULL,",");
			if(!nmea_metre_unit(token))continue;
			if (!isfinite(llh[0]) || !isfinite(llh[1]) || !isfinite(llh[2]) ||
				fabs(llh[0]) > PI/2.0 || fabs(llh[1]) > PI)
				continue;

			// Convert geodetic position into ECEF coordinates
			llh2xyz(llh, pos);

			if(have_previous) {
				while(timestamp+day_offset<previous_time-43200.0)day_offset+=86400.0;
				timestamp+=day_offset;
			}
			if(!have_previous){start_time=previous_time=timestamp;
				memcpy(previous,pos,sizeof(previous));memcpy(xyz[0],pos,sizeof(pos));
				numd=1;have_previous=1;}
			else {
				if(append_motion_segment(xyz,&numd,start_time+0.1*(double)numd,
					previous_time,previous,timestamp,pos)!=0){fclose(fp);return -2;}
				previous_time=timestamp;memcpy(previous,pos,sizeof(previous));
			}

			if (numd>=USER_MOTION_SIZE)
				break;
		}
	}

	fclose(fp);

	return (numd);
}

int generateNavMsg(gpstime_t g, channel_t *chan, int init)
{
	int iwrd,isbf;
	gpstime_t g0;
	uint32_t tow;
	uint32_t sbfwrd;
	uint32_t prevwrd;
	int nib;

	g0.week = g.week;
	g0.sec = floor(g.sec/30.0) * 30.0; // Align with the current 30-second frame.
	chan->g0 = g0; // Data bit reference time

	tow = (uint32_t)(g0.sec/6.0) % UINT32_C(100800);

	if (init==1) // Initialize subframe 5
	{
		prevwrd = 0UL;

		for (iwrd=0; iwrd<N_DWRD_SBF; iwrd++)
		{
			sbfwrd = chan->sbf[4][iwrd];

			// Add TOW-count message into HOW
			if (iwrd==1)
				sbfwrd |= ((tow&0x1FFFFUL)<<13);

			// Compute checksum
			sbfwrd |= (prevwrd<<30) & 0xC0000000UL; // 2 LSBs of the previous transmitted word
			nib = ((iwrd==1)||(iwrd==9))?1:0; // Non-information bearing bits for word 2 and 10
			chan->dwrd[iwrd] = computeChecksum(sbfwrd, nib);

			prevwrd = chan->dwrd[iwrd];
		}
	}
	else // Save subframe 5
	{
		for (iwrd=0; iwrd<N_DWRD_SBF; iwrd++)
		{
			chan->dwrd[iwrd] = chan->dwrd[N_DWRD_SBF*N_SBF+iwrd];

			prevwrd = chan->dwrd[iwrd];
		}
		/*
		// Sanity check
		if (((chan->dwrd[1])&(0x1FFFFUL<<13)) != ((tow&0x1FFFFUL)<<13))
		{
			printf("\nWARNING: Invalid TOW in subframe 5.\n");
			return(0);
		}
		*/
	}

	for (isbf=0; isbf<N_SBF; isbf++)
	{
		tow=(tow+UINT32_C(1))%UINT32_C(100800);

		for (iwrd=0; iwrd<N_DWRD_SBF; iwrd++)
		{
			sbfwrd = chan->sbf[isbf][iwrd];

			// Add TOW-count message into HOW
			if (iwrd==1)
				sbfwrd |= ((tow&0x1FFFFUL)<<13);

			// Compute checksum
			sbfwrd |= (prevwrd<<30) & 0xC0000000UL; // 2 LSBs of the previous transmitted word
			nib = ((iwrd==1)||(iwrd==9))?1:0; // Non-information bearing bits for word 2 and 10
			chan->dwrd[(isbf+1)*N_DWRD_SBF+iwrd] = computeChecksum(sbfwrd, nib);

			prevwrd = chan->dwrd[(isbf+1)*N_DWRD_SBF+iwrd];
		}
	}

	return(1);
}

int checkSatVisibility(ephem_t eph, gpstime_t g, double *xyz, double elvMask, double *azel)
{
	range_t rho;

	if (eph.vflg != 1)
		return (-1); // Invalid

	if (eph.sv_health != 0)
		return (-1); // Unhealthy

	computeRange(&rho, eph, g, xyz);
	azel[0] = rho.azel[0];
	azel[1] = rho.azel[1];

	if (azel[1]*R2D > elvMask)
		return (1); // Visible
	// else
	return (0); // Invisible
}

int allocateChannel(channel_t *chan, ephem_t *eph, gpstime_t grx, double *xyz, double elvMask)
{
	int nsat=0;
	int i,sv;
	double azel[2];

	range_t rho;
	double ref[3]={0.0};
	double r_ref,r_xyz;
	double phase_ini;

	for (sv=0; sv<MAX_SAT; sv++)
	{
		if(checkSatVisibility(eph[sv], grx, xyz, elvMask, azel)==1)
		{
			nsat++; // Number of visible satellites

			if (allocatedSat[sv]==-1) // Visible but not allocated
			{
				// Allocated new satellite
				for (i=0; i<MAX_CHAN; i++)
				{
					if (chan[i].prn==0)
					{
						// Initialize channel
						chan[i].prn = sv+1;
						chan[i].azel[0] = azel[0];
						chan[i].azel[1] = azel[1];

						// C/A code generation
						codegen(chan[i].ca, chan[i].prn);

						// Generate subframe
						eph2sbf(eph[sv], chan[i].sbf);

						// Generate navigation message
						generateNavMsg(grx, &chan[i], 1);

						// Initialize pseudorange
						computeRange(&rho, eph[sv], grx, xyz);
						chan[i].rho0 = rho;

						// Initialize carrier phase
						r_xyz = rho.range;

						computeRange(&rho, eph[sv], grx, ref);
						r_ref = rho.range;

						phase_ini = (2.0*r_ref - r_xyz)/LAMBDA_L1;
						phase_ini -= floor(phase_ini);
						chan[i].carr_phase = (unsigned int)(512 * 65536.0 * phase_ini);

						// Done.
						break;
					}
				}

				// Set satellite allocation channel
				if (i<MAX_CHAN)
					allocatedSat[sv] = i;
			}
		}
		else if (allocatedSat[sv]>=0) // Not visible but allocated
		{
			// Clear channel
			chan[allocatedSat[sv]].prn = 0;

			// Clear satellite allocation flag
			allocatedSat[sv] = -1;
		}
	}

	return(nsat);
}

#ifndef BLADE_GPS
void usage(void)
{
	printf("Usage: gps-sdr-sim [options]\n"
		"Options:\n"
		"  -e <gps_nav>     RINEX navigation file for GPS ephemerides (required)\n"
		"  -u <user_motion> User motion file (dynamic mode)\n"
		"  -g <nmea_gga>    NMEA GGA stream (dynamic mode)\n"
		"  -l <location>    Lat,Lon,Hgt (static mode) e.g. 30.286502,120.032669,100\n"
		"  -t <date,time>   Scenario start time YYYY/MM/DD,hh:mm:ss\n"
		"  -d <duration>    Duration [sec] (max: %.0f)\n"
		"  -o <output>      I/Q sampling data file (default: gpssim.bin)\n"
		"  -s <frequency>   Sampling frequency [Hz] (default: 2600000)\n"
		"  -b <iq_bits>     I/Q data format [1/8/16] (default: 16)\n"
		"  -v               Show details about simulated channels\n",
		((double)USER_MOTION_SIZE)/10.0);

	return;
}
#endif

#ifndef BLADE_GPS
int main(int argc, char *argv[])
{
#else
void *gps_task(void *arg)
{
	sim_t *s = (sim_t *)arg;
#endif

	int sv;
	int neph;
	ephem_t eph[EPHEM_ARRAY_SIZE][MAX_SAT];
	ephem_t current_eph[MAX_SAT];
	ephem_t next_eph[MAX_SAT];
	gpstime_t g0;
	
	double llh[3];
	
	int i;
	channel_t chan[MAX_CHAN];
	double elvmask = 0.0; // in degree

	int ip,qp;
	int iTable;
	short *iq_buff = NULL;
	#ifndef BLADE_GPS
	signed char *iq8_buff = NULL;
	#endif

	gpstime_t grx;
	double delt;
	int isamp;
	int found_min;
	int found_max;

	int iumd;
	int numd;
	char umfile[MAX_CHAR];
	//double xyz[USER_MOTION_SIZE][3];
	double **xyz = NULL;
	double *xyz_storage = NULL;

	int staticLocationMode = FALSE;
	int nmeaGGA = FALSE;
	int geodeticMotion = FALSE;

	char navfile[MAX_CHAR];

	int iq_buff_size;
	#ifndef BLADE_GPS
	int data_format;
	#endif

	int gain[MAX_CHAN];
	double mix_scale = 1.0;
	double path_loss;
	double ant_gain;
	double ant_pat[37];
	int ibs; // boresight angle index

	datetime_t t0,tmin,tmax;
	gpstime_t gmin,gmax;
	int igrx;

	int iduration;
	int verb;

#ifndef BLADE_GPS
	clock_t tstart,tend;
	FILE *fp;
	char outfile[MAX_CHAR];
	double samp_freq;
	double duration;
	int result;
#else
	int interactive = FALSE;
	motion_controller_t controller = {0};
	int key;
	int key_direction;
	int direction = UNDEF;
	double velocity = 0.0;
	double tmat[3][3];
	double neu[3];
#endif

	////////////////////////////////////////////////////////////
	// Read options
	////////////////////////////////////////////////////////////

#ifndef BLADE_GPS
	// Default options
	navfile[0] = 0;
	umfile[0] = 0;
	strcpy(outfile, "gpssim.bin");
	samp_freq = 2.6e6;
	data_format = SC16;
	g0.week = -1; // Invalid start time
	iduration = USER_MOTION_SIZE;
	verb = 0;

	if (argc<3)
	{
		usage();
		exit(1);
	}

	while ((result=getopt(argc,argv,"e:u:g:l:o:s:b:t:d:v"))!=-1)
	{
		switch (result)
		{
		case 'e':
			strcpy(navfile, optarg);
			break;
		case 'u':
			strcpy(umfile, optarg);
			nmeaGGA = FALSE;
			break;
		case 'g':
			strcpy(umfile, optarg);
			nmeaGGA = TRUE;
			break;
		case 'l':
			// Static geodetic coordinates input mode
			// Added by scateu@gmail.com
			staticLocationMode = TRUE;
			sscanf(optarg,"%lf,%lf,%lf",&llh[0],&llh[1],&llh[2]);
			llh[0] = llh[0] / R2D; // convert to RAD
			llh[1] = llh[1] / R2D; // convert to RAD
			break;
		case 'o':
			strcpy(outfile, optarg);
			break;
		case 's':
			samp_freq = atof(optarg);
			if (samp_freq<1.0e6)
			{
				printf("ERROR: Invalid sampling frequency.\n");
				exit(1);
			}
			break;
		case 'b':
			data_format = atoi(optarg);
			if (data_format!=SC01 && data_format!=SC08 && data_format!=SC16)
			{
				printf("ERROR: Invalid I/Q data format.\n");
				exit(1);
			}
			break;
		case 't':
			sscanf(optarg, "%d/%d/%d,%d:%d:%lf", &t0.y, &t0.m, &t0.d, &t0.hh, &t0.mm, &t0.sec);
			if (t0.y<=1980 || t0.m<1 || t0.m>12 || t0.d<1 || t0.d>31 ||
				t0.hh<0 || t0.hh>23 || t0.mm<0 || t0.mm>59 || t0.sec<0.0 || t0.sec>=60.0)
			{
				printf("ERROR: Invalid date and time.\n");
				exit(1);
			}
			t0.sec = floor(t0.sec);
			date2gps(&t0, &g0);
			break;
		case 'd':
			duration = atof(optarg);
			if (duration<0.0 || duration>((double)USER_MOTION_SIZE)/10.0)
			{
				printf("ERROR: Invalid duration.\n");
				exit(1);
			}
			iduration = (int)(duration*10.0+0.5);
			break;
		case 'v':
			verb = 1;
			break;
		case ':':
		case '?':
			usage();
			exit(1);
		default:
			break;
		}
	}

	if (navfile[0]==0)
	{
		printf("ERROR: GPS ephemeris file is not specified.\n");
		exit(1);
	}

	if (umfile[0]==0 && !staticLocationMode)
	{
		printf("ERROR: User motion file / NMEA GGA stream is not specified.\n");
		printf("You may use -l to specify the static location directly.\n");
		exit(1);
	}

	// Buffer size	
	samp_freq = floor(samp_freq/10.0);
	iq_buff_size = (int)samp_freq; // samples per 0.1sec
	samp_freq *= 10.0;

	delt = 1.0/samp_freq;
#else
	strcpy(navfile, s->opt.navfile);
	strcpy(umfile, s->opt.umfile);
	
	staticLocationMode = s->opt.staticLocationMode;
	llh[0] = s->opt.llh[0];
	llh[1] = s->opt.llh[1];
	llh[2] = s->opt.llh[2];
	
	g0.week = s->opt.g0.week;
	g0.sec = s->opt.g0.sec;
	gps2date(&g0, &t0);

	nmeaGGA = s->opt.nmeaGGA;
	geodeticMotion = s->opt.geodeticMotion;

	iduration = s->opt.iduration;
	verb = s->opt.verb;
	elvmask = s->opt.elevation_mask;

	iq_buff_size = (int)s->iq_block_samples;

	delt = 1.0/(double)s->opt.tx_sample_rate;

	interactive = s->opt.interactive;
#endif

	////////////////////////////////////////////////////////////
	// Receiver position
	////////////////////////////////////////////////////////////

	// Allocate row pointers plus one contiguous motion-data block.
	xyz = (double **)malloc(USER_MOTION_SIZE * sizeof(*xyz));
	xyz_storage = (double *)calloc(USER_MOTION_SIZE * 3U, sizeof(*xyz_storage));

	if (xyz==NULL || xyz_storage==NULL)
	{
		free(xyz);
		free(xyz_storage);
		xyz = NULL;
		xyz_storage = NULL;
		printf("ERROR: Failed to allocate user motion array.\n");
#ifndef BLADE_GPS
		exit(1);
#else
		goto exit;
#endif
	}

	for (i=0; i<USER_MOTION_SIZE; i++)
		xyz[i] = xyz_storage + (size_t)i * 3U;

	if (!staticLocationMode)
	{
		// Read user motion file
		if (nmeaGGA==TRUE)
			numd = readNmeaGGA(xyz, umfile);
		else if (geodeticMotion==TRUE)
			numd = readLlhMotion(xyz, umfile);
		else
		{
			numd = readUserMotion(xyz, umfile);
		}

		if (numd==-1)
		{
			printf("ERROR: Failed to open user motion / NMEA GGA file.\n");
#ifndef BLADE_GPS
			exit(1);
#else
			goto exit;
#endif
		}
		else if (numd<=0)
		{
			printf("ERROR: Failed to read user motion / NMEA GGA data.\n");
#ifndef BLADE_GPS
			exit(1);
#else
			goto exit;
#endif
		}

		// Motion files provide ECEF positions; derive LLH for status output and
		// for the local tangent frame used when interactive motion is layered on.
		xyz2llh(xyz[0], llh);

		// Set simulation duration
		if (numd>iduration)
			numd = iduration;
	} 
	else 
	{
		// Static geodetic coordinates input mode: "-l"
		// Added by scateu@gmail.com 
		printf("Using static location mode.\n");
		llh2xyz(llh,xyz[0]); // Convert llh to xyz

		numd = iduration;
		
		for (iumd=1; iumd<numd; iumd++)
		{
			xyz[iumd][0] = xyz[0][0];
			xyz[iumd][1] = xyz[0][1];
			xyz[iumd][2] = xyz[0][2];
		}
	}

#ifdef BLADE_GPS
	// Initialize the local tangential matrix for interactive mode
	ltcmat(llh, tmat);
	if (interactive)
	{
		printf("Enable interactive mode.\n");
		numd = iduration;
	}
	if(s->opt.controller_index>=0) {
		if(motion_controller_open(&controller,s->opt.controller_index)!=0) {
			fprintf(stderr,"ERROR: Cannot open requested SDL game controller.\n");
			goto exit;
		}
		printf("Enable live game-controller receiver motion.\n");
		numd=iduration;
	}
#endif

	printf("xyz = %11.1f, %11.1f, %11.1f\n", xyz[0][0], xyz[0][1], xyz[0][2]);
	printf("llh = %11.6f, %11.6f, %11.1f\n", llh[0]*R2D, llh[1]*R2D, llh[2]);

	////////////////////////////////////////////////////////////
	// Read ephemeris
	////////////////////////////////////////////////////////////

	neph = readRinexNavAll(eph, navfile);

	if (neph<=0)
	{
		printf("ERROR: No ephemeris available.\n");
#ifndef BLADE_GPS
		exit(1);
#else
		goto exit;
#endif
	}

	found_min = 0;
	found_max = 0;
	for (i=0; i<neph; i++) {
		for (sv=0; sv<MAX_SAT; sv++) {
			if (eph[i][sv].vflg != 1)
				continue;

			if (!found_min || subGpsTime(eph[i][sv].toc, gmin) < 0.0) {
				gmin = eph[i][sv].toc;
				tmin = eph[i][sv].t;
				found_min = 1;
			}
			if (!found_max || subGpsTime(eph[i][sv].toc, gmax) > 0.0) {
				gmax = eph[i][sv].toc;
				tmax = eph[i][sv].t;
				found_max = 1;
			}
		}
	}

	if (!found_min || !found_max)
	{
		printf("ERROR: No valid ephemeris records found.\n");
#ifndef BLADE_GPS
		exit(1);
#else
		goto exit;
#endif
	}

	if (g0.week>=0)
	{
		if (subGpsTime(g0, gmin)<0.0 || subGpsTime(gmax, g0)<0.0)
		{
			printf("ERROR: Invalid start time.\n");
			printf("tmin = %4d/%02d/%02d,%02d:%02d:%02.0f (%d:%.0f)\n", 
				tmin.y, tmin.m, tmin.d, tmin.hh, tmin.mm, tmin.sec,
				gmin.week, gmin.sec);
			printf("tmax = %4d/%02d/%02d,%02d:%02d:%02.0f (%d:%.0f)\n", 
				tmax.y, tmax.m, tmax.d, tmax.hh, tmax.mm, tmax.sec,
				gmax.week, gmax.sec);
#ifndef BLADE_GPS
			exit(1);
#else
			goto exit;
#endif
		}
	}
	else
	{
		g0 = gmin;
		t0 = tmin;
	}

	printf("Start time = %4d/%02d/%02d,%02d:%02d:%02.0f (%d:%.0f)\n", 
		t0.y, t0.m, t0.d, t0.hh, t0.mm, t0.sec, g0.week, g0.sec);
	printf("Duration = %.1f [sec]\n", ((double)numd)/10.0);

	// Select the closest valid record independently for every satellite.
	if (selectEphemerides(current_eph, eph, neph, g0) == 0)
	{
		printf("ERROR: No current ephemerides were found within the supported fit interval.\n");
#ifndef BLADE_GPS
		exit(1);
#else
		goto exit;
#endif
	}

	////////////////////////////////////////////////////////////
	// Baseband signal buffer and output file
	////////////////////////////////////////////////////////////

	// Allocate I/Q buffer
	iq_buff = calloc((size_t)2U*(size_t)iq_buff_size, sizeof(*iq_buff));

	if (iq_buff==NULL)
	{
		printf("ERROR: Failed to allocate 16-bit I/Q buffer.\n");
#ifndef BLADE_GPS
		exit(1);
#else
		goto exit;
#endif
	}

#ifndef BLADE_GPS // SC16 for bladeRF
	if (data_format==SC08)
	{
		iq8_buff = calloc(2*iq_buff_size, 1);
		if (iq8_buff==NULL)
		{
			printf("ERROR: Failed to allocate 8-bit I/Q buffer.\n");
			exit(1);
		}
	}
	else if (data_format==SC01)
	{
		iq8_buff = calloc(iq_buff_size/4, 1); // byte = {I0, Q0, I1, Q1, I2, Q2, I3, Q3}
		if (iq8_buff==NULL)
		{
			printf("ERROR: Failed to allocate compressed 1-bit I/Q buffer.\n");
			exit(1);
		}
	}
#endif

#ifndef BLADE_GPS // No output file
	// Open output file
	if (NULL==(fp=fopen(outfile,"wb")))
	{
		printf("ERROR: Failed to open output file.\n");
		exit(1);
	}
#endif
	////////////////////////////////////////////////////////////
	// Initialize channels
	////////////////////////////////////////////////////////////

	// Clear all channels
	memset(chan, 0, sizeof(chan));

	// Clear satellite allocation flag
	for (sv=0; sv<MAX_SAT; sv++)
		allocatedSat[sv] = -1;

	// Initial reception time
	grx = g0;

	// Allocate visible satellites
	allocateChannel(chan, current_eph, grx, xyz[0], elvmask);

	for(i=0; i<MAX_CHAN; i++)
	{
		if (chan[i].prn>0)
			printf("%02d %6.1f %5.1f %11.1f\n", chan[i].prn, 
				chan[i].azel[0]*R2D, chan[i].azel[1]*R2D, chan[i].rho0.d);
	}

	////////////////////////////////////////////////////////////
	// Receiver antenna gain pattern
	////////////////////////////////////////////////////////////

	for (i=0; i<37; i++)
		ant_pat[i] = pow(10.0, -ant_pat_db[i]/20.0);

	////////////////////////////////////////////////////////////
	// Generate baseband signals
	////////////////////////////////////////////////////////////

#ifndef BLADE_GPS
	tstart = clock();
#endif

	// Update receiver time
	grx.sec += 0.1;
	normalizeGpsTime(&grx);

	for (iumd=0; iumd<numd; iumd++)
	{
#ifdef BLADE_GPS
		if (stop_was_requested()) {
			printf("\nStop requested; finishing buffered transmission.\n");
			goto cleanup;
		}
#endif
#ifdef BLADE_GPS
		if(iumd>0&&controller.active) {
			memcpy(xyz[iumd],xyz[iumd-1],3U*sizeof(double));
		}
		if (interactive)
		{
			key_direction = UNDEF;

			if(_kbhit())
			{
				key = _getch();
				switch (key)
				{
				case NORTH_KEY:
					key_direction = NORTH;
					break;
				case SOUTH_KEY:
					key_direction = SOUTH;
					break;
				case EAST_KEY:
					key_direction = EAST;
					break;
				case WEST_KEY:
					key_direction = WEST;
					break;
				case UP_KEY:
					key_direction = UP;
					break;
				case DOWN_KEY:
					key_direction = DOWN;
					break;
				default:
					break;
				}
			}

			if ((key_direction!=UNDEF)&&(direction==key_direction))
			{
				// Accelerate toward the key direction
				if (velocity<MAX_VEL)
					velocity += DEL_VEL;
			}
			else
			{
				// Deaccelerate and stop
				if (velocity>=0.0)
					velocity -= DEL_VEL;
				else
					direction = key_direction; // then change the direction
			}

			// Stay at the current location until a direction is active.
			if (iumd > 0) {
				xyz[iumd][0] = xyz[iumd-1][0];
				xyz[iumd][1] = xyz[iumd-1][1];
				xyz[iumd][2] = xyz[iumd-1][2];
			}

			if (iumd > 0 && (direction!=UNDEF)&&(velocity>=0.0))
			{
				// Update the user location
				neu[0] = 0.0;
				neu[1] = 0.0;
				neu[2] = 0.0;

				// Refresh the local frame as the simulated receiver moves.
				xyz2llh(xyz[iumd-1], llh);
				ltcmat(llh, tmat);

				switch(direction)
				{
				case NORTH:
					neu[0] = velocity * 0.1;
					break;
				case SOUTH:
					neu[0] = -velocity * 0.1;
					break;
				case EAST:
					neu[1] = velocity * 0.1;
					break;
				case WEST:
					neu[1] = -velocity * 0.1;
					break;
				case UP:
					neu[2] = velocity * 0.1;
					break;
				case DOWN:
					neu[2] = -velocity * 0.1;
					break;
				default:
					break;
				}

				xyz[iumd][0] += tmat[0][0]*neu[0] + tmat[1][0]*neu[1] + tmat[2][0]*neu[2];
				xyz[iumd][1] += tmat[0][1]*neu[0] + tmat[1][1]*neu[1] + tmat[2][1]*neu[2];
				xyz[iumd][2] += tmat[0][2]*neu[0] + tmat[1][2]*neu[1] + tmat[2][2]*neu[2];
			}
		}
		if(iumd>0&&controller.active) {
			double controller_neu[3];
			if(motion_controller_poll(&controller,MAX_VEL,MAX_VEL,controller_neu)!=0) {
				fprintf(stderr,"ERROR: Game controller disconnected.\n");
				goto cleanup;
			}
			xyz2llh(xyz[iumd-1],llh);ltcmat(llh,tmat);
			xyz[iumd][0]+=0.1*(tmat[0][0]*controller_neu[0]+tmat[1][0]*controller_neu[1]+tmat[2][0]*controller_neu[2]);
			xyz[iumd][1]+=0.1*(tmat[0][1]*controller_neu[0]+tmat[1][1]*controller_neu[1]+tmat[2][1]*controller_neu[2]);
			xyz[iumd][2]+=0.1*(tmat[0][2]*controller_neu[0]+tmat[1][2]*controller_neu[1]+tmat[2][2]*controller_neu[2]);
		}
#endif
		for (i=0; i<MAX_CHAN; i++)
		{
			if (chan[i].prn>0)
			{
				// Refresh code phase and data bit counters
				int sv = chan[i].prn-1;
				range_t rho;

				// Current pseudorange
				computeRange(&rho, current_eph[sv], grx, xyz[iumd]);
				chan[i].azel[0] = rho.azel[0];
				chan[i].azel[1] = rho.azel[1];

				// Update code phase and data bit counters
				computeCodePhase(&chan[i], rho, 0.1);
				chan[i].carr_phasestep = (int)(512 * 65536.0 * chan[i].f_carr * delt);

				// Path loss
				path_loss = 20200000.0/rho.d;

				// Receiver antenna gain
				ibs = (int)((90.0-rho.azel[1]*R2D)/5.0); // convert elevation to boresight
				if (ibs < 0)
					ibs = 0;
				else if (ibs > 36)
					ibs = 36;
				ant_gain = ant_pat[ibs];

				// Signal gain
				gain[i] = (int)(path_loss*ant_gain*100.0); // scaled by 100
			}
		}
		{
			double worst_component=0.0;
			for(i=0;i<MAX_CHAN;i++) if(chan[i].prn>0)
				worst_component+=2.5*fabs((double)gain[i]);
			mix_scale=worst_component>1800.0?1800.0/worst_component:1.0;
		}

		for (isamp=0; isamp<iq_buff_size; isamp++)
		{
			int i_acc = 0;
			int q_acc = 0;

			for (i=0; i<MAX_CHAN; i++)
			{
				if (chan[i].prn>0)
				{
					iTable = (chan[i].carr_phase >> 16) & 511;

					ip = chan[i].dataBit * chan[i].codeCA * cosTable512[iTable] * gain[i];
					qp = chan[i].dataBit * chan[i].codeCA * sinTable512[iTable] * gain[i];

					i_acc += (ip >= 0 ? ip + 50 : ip - 50)/100;
					q_acc += (qp >= 0 ? qp + 50 : qp - 50)/100;

					// Update code phase
					chan[i].code_phase += chan[i].f_code * delt;

					if (chan[i].code_phase>=CA_SEQ_LEN)
					{
						chan[i].code_phase -= CA_SEQ_LEN;

						chan[i].icode++;
					
						if (chan[i].icode>=20) // 20 C/A codes = 1 navigation data bit
						{
							chan[i].icode = 0;
							chan[i].ibit++;
						
							if (chan[i].ibit>=30) // 30 navigation data bits = 1 word
							{
								chan[i].ibit = 0;
								chan[i].iword++;
								/*
								if (chan[i].iword>=N_DWRD)
									printf("\nWARNING: Subframe word buffer overflow.\n");
								*/
							}

							// Set new navigation data bit
					chan[i].dataBit = (int)((chan[i].dwrd[chan[i].iword]>>(29-chan[i].ibit)) & UINT32_C(1))*2-1;
						}
					}

					// Set currnt code chip
					chan[i].codeCA = chan[i].ca[(int)chan[i].code_phase]*2-1;

					// Update carrier phase
					chan[i].carr_phase += (uint32_t)chan[i].carr_phasestep;
				}
			}

			// Store I/Q samples into buffer
			iq_buff[isamp*2] = (short)lrint((double)i_acc*mix_scale);
			iq_buff[isamp*2+1] = (short)lrint((double)q_acc*mix_scale);

		} // End of omp parallel for

#ifndef BLADE_GPS
		if (data_format==SC01)
		{
			for (isamp=0; isamp<2*iq_buff_size; isamp++)
			{
				if (isamp%8==0)
					iq8_buff[isamp/8] = 0x00;

				iq8_buff[isamp/8] |= (iq_buff[isamp]>0?0x01:0x00)<<(7-isamp%8);
			}

			fwrite(iq8_buff, 1, iq_buff_size/4, fp);
		}
		else if (data_format==SC08)
		{
			for (isamp=0; isamp<2*iq_buff_size; isamp++)
				iq8_buff[isamp] = iq_buff[isamp]>>4; // 12-bit bladeRF -> 8-bit HackRF

			fwrite(iq8_buff, 1, 2*iq_buff_size, fp);
		} 
		else // data_format==SC16
		{
			/*
			for (isamp=0; isamp<2*iq_buff_size; isamp++)
				iq_buff[isamp] = iq_buff[isamp]>0?1000:-1000; // Emulated 1-bit I/Q
			*/
			fwrite(iq_buff, 2, 2*iq_buff_size, fp);
		}
#else
		////////////////////////////////////////////////////////////
		// Write into FIFO
		///////////////////////////////////////////////////////////

		pthread_mutex_lock(&(s->gps.lock));
		if (!s->gps.ready) {
			// Initialization has been done. Ready to create TX task.
			printf("GPS signal generator is ready!\n");
			s->gps.ready = 1;
			pthread_cond_signal(&(s->gps.initialization_done));
		}

		// Wait until FIFO writing is ready.
		while (!is_fifo_write_ready(s) && !s->finished)
			pthread_cond_wait(&(s->fifo_write_ready), &(s->gps.lock));
		if (s->finished) {
			pthread_mutex_unlock(&(s->gps.lock));
			goto cleanup;
		}

		// Write into FIFO
		memcpy(&(s->fifo[s->head * 2]), iq_buff, s->iq_block_samples * 2 * sizeof(short));

		s->head += (long)s->iq_block_samples;
		if ((size_t)s->head >= s->fifo_length)
			s->head -= (long)s->fifo_length;
		pthread_cond_signal(&(s->fifo_read_ready));
		pthread_mutex_unlock(&(s->gps.lock));
#endif
		//
		// Update navigation message and channel allocation every 30 seconds
		//

		igrx = (int)(grx.sec*10.0+0.5);

		if (igrx%300==0) // Every 30 seconds
		{
			int available = selectEphemerides(next_eph, eph, neph, grx);

			if (available == 0) {
				fprintf(stderr, "\nERROR: Ephemeris coverage ended at GPS week %d, %.1f seconds.\n",
					grx.week, grx.sec);
#ifdef BLADE_GPS
				s->gps.error = -1;
				goto cleanup;
#else
				break;
#endif
			}

			// Update navigation message
			for (i=0; i<MAX_CHAN; i++) {
				if (chan[i].prn>0) {
					sv = chan[i].prn - 1;
					if (next_eph[sv].vflg == 1 &&
						(current_eph[sv].iode != next_eph[sv].iode ||
						 current_eph[sv].iodc != next_eph[sv].iodc ||
						 subGpsTime(current_eph[sv].toe, next_eph[sv].toe) != 0.0)) {
						eph2sbf(next_eph[sv], chan[i].sbf);
						/* Re-evaluate the last range with the replacement orbit so
						 * the next Doppler estimate does not interpret an ephemeris
						 * representation change as receiver velocity. */
						computeRange(&chan[i].rho0, next_eph[sv], chan[i].rho0.g, xyz[iumd]);
					}
					generateNavMsg(grx, &chan[i], 0);
				}
			}

			memcpy(current_eph, next_eph, sizeof(current_eph));

			// Update channel allocation
			allocateChannel(chan, current_eph, grx, xyz[iumd], elvmask);

			// Show details about simulated channels.
			if (verb)
			{
				printf("\n");
				gps2date(&grx, &t0);
				printf("%4d/%02d/%02d,%02d:%02d:%02.0f (%d:%.0f)\n", 
					t0.y, t0.m, t0.d, t0.hh, t0.mm, t0.sec, grx.week, grx.sec);
				printf("xyz = %11.1f, %11.1f, %11.1f\n", xyz[iumd][0], xyz[iumd][1], xyz[iumd][2]);
				xyz2llh(xyz[iumd],llh);
				printf("llh = %11.6f, %11.6f, %11.1f\n", llh[0]*R2D, llh[1]*R2D, llh[2]);
				for (i=0; i<MAX_CHAN; i++)
				{
					if (chan[i].prn>0)
						printf("%02d %6.1f %5.1f %11.1f\n", chan[i].prn,
							chan[i].azel[0]*R2D, chan[i].azel[1]*R2D, chan[i].rho0.d);
				}
			}
		}

		// Update receiver time
		grx.sec += 0.1;
		normalizeGpsTime(&grx);

		// Update time counter
		printf("\rTime into run = %4.1f", ((double)(iumd + 1))/10.0);
		fflush(stdout);
	}

	// Done!
	pthread_mutex_lock(&(s->gps.lock));
	s->finished = true;
	pthread_cond_broadcast(&(s->fifo_read_ready));
	pthread_cond_broadcast(&(s->fifo_write_ready));
	pthread_mutex_unlock(&(s->gps.lock));

cleanup:
#ifdef BLADE_GPS
	motion_controller_close(&controller);
#endif
	// Free I/Q buffer
	free(iq_buff);
	iq_buff = NULL;

	// Free user motion array
	if (xyz != NULL) {
		free(xyz_storage);
		free(xyz);
		xyz = NULL;
		xyz_storage = NULL;
	}

#ifndef BLADE_GPS
	free(iq8_buff);

	// Close file
	fclose(fp);

	tend = clock();

	printf("\nDone!\n");

	// Process time
	printf("Process time = %.1f [sec]\n", (double)(tend-tstart)/CLOCKS_PER_SEC);

	return(0);
#else
exit:
	motion_controller_close(&controller);
	free(iq_buff);
	if (xyz != NULL) {
		free(xyz_storage);
		free(xyz);
	}
	pthread_mutex_lock(&(s->gps.lock));
	if (!s->gps.ready && !s->finished)
		s->gps.error = -1;
	s->finished = true;
	s->gps.ready = 1;
	pthread_cond_broadcast(&(s->gps.initialization_done));
	pthread_cond_broadcast(&(s->fifo_read_ready));
	pthread_cond_broadcast(&(s->fifo_write_ready));
	pthread_mutex_unlock(&(s->gps.lock));
	return (NULL);
#endif
}
