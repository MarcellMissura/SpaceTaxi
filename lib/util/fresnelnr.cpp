#include "lib/globals.h"
#include "fresnelnr.h"
#include <stdio.h>

fcomplex Cadd(fcomplex a, fcomplex b)
{
	fcomplex c;
	c.r=a.r+b.r;
	c.i=a.i+b.i;
	return c;
}

fcomplex Csub(fcomplex a, fcomplex b)
{
	fcomplex c;
	c.r=a.r-b.r;
	c.i=a.i-b.i;
	return c;
}


fcomplex Cmul(fcomplex a, fcomplex b)
{
	fcomplex c;
	c.r=a.r*b.r-a.i*b.i;
	c.i=a.i*b.r+a.r*b.i;
	return c;
}

fcomplex Complex(float re, float im)
{
	fcomplex c;
	c.r=re;
	c.i=im;
	return c;
}

fcomplex Conjg(fcomplex z)
{
	fcomplex c;
	c.r=z.r;
	c.i = -z.i;
	return c;
}

fcomplex Cdiv(fcomplex a, fcomplex b)
{
	fcomplex c;
	float r,den;
	if (fabs(b.r) >= fabs(b.i)) {
		r=b.i/b.r;
		den=b.r+r*b.i;
		c.r=(a.r+r*a.i)/den;
		c.i=(a.i-r*a.r)/den;
	} else {
		r=b.r/b.i;
		den=b.i+r*b.r;
		c.r=(a.r*r+a.i)/den;
		c.i=(a.i*r-a.r)/den;
	}
	return c;
}

float Cabs(fcomplex z)
{
	float x,y,ans,temp;
	x=fabs(z.r);
	y=fabs(z.i);
	if (x == 0.0)
		ans=y;
	else if (y == 0.0)
		ans=x;
	else if (x > y) {
		temp=y/x;
		ans=x*sqrt(1.0+temp*temp);
	} else {
		temp=x/y;
		ans=y*sqrt(1.0+temp*temp);
	}
	return ans;
}

fcomplex Csqrt(fcomplex z)
{
	fcomplex c;
	float x,y,w,r;
	if ((z.r == 0.0) && (z.i == 0.0)) {
		c.r=0.0;
		c.i=0.0;
		return c;
	} else {
		x=fabs(z.r);
		y=fabs(z.i);
		if (x >= y) {
			r=y/x;
			w=sqrt(x)*sqrt(0.5*(1.0+sqrt(1.0+r*r)));
		} else {
			r=x/y;
			w=sqrt(y)*sqrt(0.5*(r+sqrt(1.0+r*r)));
		}
		if (z.r >= 0.0) {
			c.r=w;
			c.i=z.i/(2.0*w);
		} else {
			c.i=(z.i >= 0) ? w : -w;
			c.r=z.i/(2.0*c.i);
		}
		return c;
	}
}

fcomplex RCmul(float x, fcomplex a)
{
	fcomplex c;
	c.r=x*a.r;
	c.i=x*a.i;
	return c;
}

void fresnel(double x, double *s, double *c)
{
	int k,n,odd;
	float a,ax,fact,pix2,sign,sum,sumc,sums,term,test;
	fcomplex b,cc,d,h,del,cs;
	ax=fabs(x);
	if (ax < sqrt(FPMIN))
	{
		*s=0.0;
		*c=ax;
	}
	else if (ax <= 1.5)
	{
		sum=sums=0.0;
		sumc=ax;
		sign=1.0;
		fact=PI2*ax*ax;
		odd=1;
		term=ax;
		n=3;
		for (k=1;k<=MAXIT;k++)
		{
			term *= fact/k;
			sum += sign*term/n;
			test=fabs(sum)*EPS;
			if (odd) {
				sign = -sign;
				sums=sum;
				sum=sumc;
			} else {
				sumc=sum;
				sum=sums;
			}
			if (term < test) break;
			odd=!odd;
			n += 2;
		}
		if (k > MAXIT) printf("series failed in frenel\n");
		*s=sums;
		*c=sumc;
	}
	else
	{
		pix2=PI*ax*ax;
		b=Complex(1.0,-pix2);
		cc=Complex(1.0/FPMIN,0.0);
		d=h=Cdiv(ONE,b);
		n = -1;
		for (k=2;k<=MAXIT;k++)
		{
			n += 2;
			a = -n*(n+1);
			b=Cadd(b,Complex(4.0,0.0));
			d=Cdiv(ONE,Cadd(RCmul(a,d),b));
			cc=Cadd(b,Cdiv(Complex(a,0.0),cc));
			del=Cmul(cc,d);
			h=Cmul(h,del);
			if (fabs(del.r-1.0)+fabs(del.i) < EPS) break;
		}
		if (k > MAXIT) printf("cf failed in frenel %f\n", x);
		h=Cmul(Complex(ax,-ax),h);
        cs=Cmul(Complex(0.5,0.5),Csub(ONE,Cmul(Complex(fcos(0.5*pix2),fsin(0.5*pix2)),h)));
		*c=cs.r;
		*s=cs.i;
	}
	if (x < 0.0)
	{
		*c = -(*c);
		*s = -(*s);
	}
}
