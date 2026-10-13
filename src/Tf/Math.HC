//Math: rigid transforms (translation + unit quaternion), the way tf2
//does them. RosTfCompose(out,a,b) = a*b: apply b, then a.
//All functions accept out aliasing an input.

class CRosTf
{
  F64 x,y,z;		//translation
  F64 qx,qy,qz,qw;	//rotation
};

U0 RosTfIdentity(CRosTf *t)
{
  t->x=0.0;
  t->y=0.0;
  t->z=0.0;
  t->qx=0.0;
  t->qy=0.0;
  t->qz=0.0;
  t->qw=1.0;
}

U0 RosTfSetXyzQ(CRosTf *t,F64 x,F64 y,F64 z,F64 qx,F64 qy,F64 qz,F64 qw)
{
  t->x=x;
  t->y=y;
  t->z=z;
  t->qx=qx;
  t->qy=qy;
  t->qz=qz;
  t->qw=qw;
}

U0 RosTfNormalize(CRosTf *t)
{
  F64 n=Sqrt(t->qx*t->qx+t->qy*t->qy+t->qz*t->qz+t->qw*t->qw);
  if (n<1e-12) {
    t->qx=0.0;
    t->qy=0.0;
    t->qz=0.0;
    t->qw=1.0;
    return;
  }
  t->qx/=n;
  t->qy/=n;
  t->qz/=n;
  t->qw/=n;
}

//Rotate (x,y,z) by t's quaternion, in place.
U0 RosTfRotate(CRosTf *t,F64 *x,F64 *y,F64 *z)
{
  F64 cx,cy,cz,ccx,ccy,ccz,vx=*x,vy=*y,vz=*z;
  //v' = v + 2w(q x v) + 2 q x (q x v)
  cx=t->qy*vz-t->qz*vy;
  cy=t->qz*vx-t->qx*vz;
  cz=t->qx*vy-t->qy*vx;
  ccx=t->qy*cz-t->qz*cy;
  ccy=t->qz*cx-t->qx*cz;
  ccz=t->qx*cy-t->qy*cx;
  *x=vx+2.0*(t->qw*cx+ccx);
  *y=vy+2.0*(t->qw*cy+ccy);
  *z=vz+2.0*(t->qw*cz+ccz);
}

//Map a point from t's child frame into its parent frame, in place.
U0 RosTfApply(CRosTf *t,F64 *x,F64 *y,F64 *z)
{
  RosTfRotate(t,x,y,z);
  *x=*x+t->x;
  *y=*y+t->y;
  *z=*z+t->z;
}

//out = a*b
U0 RosTfCompose(CRosTf *out,CRosTf *a,CRosTf *b)
{
  F64 x=b->x,y=b->y,z=b->z,qx,qy,qz,qw;
  RosTfRotate(a,&x,&y,&z);
  qw=a->qw*b->qw-a->qx*b->qx-a->qy*b->qy-a->qz*b->qz;
  qx=a->qw*b->qx+a->qx*b->qw+a->qy*b->qz-a->qz*b->qy;
  qy=a->qw*b->qy-a->qx*b->qz+a->qy*b->qw+a->qz*b->qx;
  qz=a->qw*b->qz+a->qx*b->qy-a->qy*b->qx+a->qz*b->qw;
  out->x=a->x+x;
  out->y=a->y+y;
  out->z=a->z+z;
  out->qx=qx;
  out->qy=qy;
  out->qz=qz;
  out->qw=qw;
}

U0 RosTfInverse(CRosTf *out,CRosTf *a)
{
  CRosTf q;
  F64 x=-a->x,y=-a->y,z=-a->z;
  q.qx=-a->qx;
  q.qy=-a->qy;
  q.qz=-a->qz;
  q.qw=a->qw;
  RosTfRotate(&q,&x,&y,&z);
  out->x=x;
  out->y=y;
  out->z=z;
  out->qx=q.qx;
  out->qy=q.qy;
  out->qz=q.qz;
  out->qw=q.qw;
}

//Interpolate: translation linearly, rotation by slerp (r in [0,1]).
U0 RosTfLerp(CRosTf *out,CRosTf *a,CRosTf *b,F64 r)
{
  F64 dot=a->qx*b->qx+a->qy*b->qy+a->qz*b->qz+a->qw*b->qw,bs=1.0,th0,th,s0,s1,sn;
  CRosTf t;
  if (dot<0.0) {		//take the short way round
    dot=-dot;
    bs=-1.0;
  }
  if (dot>0.9995) {		//nearly equal: normalized lerp
    s0=1.0-r;
    s1=r*bs;
  } else {
    th0=Arg(dot,Sqrt(1.0-dot*dot));	//acos(dot)
    th=th0*r;
    sn=Sin(th0);
    s0=Cos(th)-dot*Sin(th)/sn;
    s1=Sin(th)/sn*bs;
  }
  t.x=a->x+(b->x-a->x)*r;
  t.y=a->y+(b->y-a->y)*r;
  t.z=a->z+(b->z-a->z)*r;
  t.qx=s0*a->qx+s1*b->qx;
  t.qy=s0*a->qy+s1*b->qy;
  t.qz=s0*a->qz+s1*b->qz;
  t.qw=s0*a->qw+s1*b->qw;
  RosTfNormalize(&t);
  MemCpy(out,&t,sizeof(CRosTf));
}

//Rotation from roll/pitch/yaw (radians, applied as tf2 does: yaw, then pitch, then roll).
U0 RosTfSetRpy(CRosTf *t,F64 roll,F64 pitch,F64 yaw)
{
  F64 cr=Cos(roll/2.0),sr=Sin(roll/2.0),cp=Cos(pitch/2.0),sp=Sin(pitch/2.0);
  F64 cy=Cos(yaw/2.0),sy=Sin(yaw/2.0);
  t->qw=cr*cp*cy+sr*sp*sy;
  t->qx=sr*cp*cy-cr*sp*sy;
  t->qy=cr*sp*cy+sr*cp*sy;
  t->qz=cr*cp*sy-sr*sp*cy;
}

U0 RosTfGetRpy(CRosTf *t,F64 *roll,F64 *pitch,F64 *yaw)
{
  F64 s=2.0*(t->qw*t->qy-t->qz*t->qx);
  *roll=Arg(1.0-2.0*(t->qx*t->qx+t->qy*t->qy),2.0*(t->qw*t->qx+t->qy*t->qz));
  if (s>=1.0)
    *pitch=pi/2.0;
  else if (s<=-1.0)
    *pitch=-pi/2.0;
  else
    *pitch=Arg(Sqrt(1.0-s*s),s);	//asin(s)
  *yaw=Arg(1.0-2.0*(t->qy*t->qy+t->qz*t->qz),2.0*(t->qw*t->qz+t->qx*t->qy));
}
