//Canvas: a pixel buffer in TempleOS's 16 colors, one byte per pixel
//(the same layout as a CDC body), so drawing is plain HolyC that also
//runs (and is tested) on Linux.
//
//  CRosCanvas *cv=RosCanvasNew(640,480);
//  RosCanvasClear(cv,ROS_BLACK);
//  RosCanvasLine(cv,0,0,639,479,ROS_YELLOW);
//  RosCanvasPpm(cv,&len);         //as a .ppm image, for looking at on Linux

#define ROS_BLACK	0
#define ROS_BLUE	1
#define ROS_GREEN	2
#define ROS_CYAN	3
#define ROS_RED		4
#define ROS_PURPLE	5
#define ROS_BROWN	6
#define ROS_LTGRAY	7
#define ROS_DKGRAY	8
#define ROS_LTBLUE	9
#define ROS_LTGREEN	10
#define ROS_LTCYAN	11
#define ROS_LTRED	12
#define ROS_LTPURPLE	13
#define ROS_YELLOW	14
#define ROS_WHITE	15

//The standard 16-color (VGA) palette TempleOS uses, as 0xRRGGBB.
I64 ros_palette[16]={
  0x000000,0x0000AA,0x00AA00,0x00AAAA,0xAA0000,0xAA00AA,0xAA5500,0xAAAAAA,
  0x555555,0x5555FF,0x55FF55,0x55FFFF,0xFF5555,0xFF55FF,0xFFFF55,0xFFFFFF};

class CRosCanvas
{
  I64 w,h;
  U8 *px;	//w*h color numbers, row by row from the top
};

CRosCanvas *RosCanvasNew(I64 w,I64 h)
{
  CRosCanvas *cv=CAlloc(sizeof(CRosCanvas));
  cv->w=w;
  cv->h=h;
  cv->px=CAlloc(w*h);
  return cv;
}

U0 RosCanvasDel(CRosCanvas *cv)
{
  if (!cv)
    return;
  Free(cv->px);
  Free(cv);
}

U0 RosCanvasClear(CRosCanvas *cv,I64 color)
{
  MemSet(cv->px,color,cv->w*cv->h);
}

U0 RosCanvasPlot(CRosCanvas *cv,I64 x,I64 y,I64 color)
{
  if (x>=0 && y>=0 && x<cv->w && y<cv->h)
    cv->px[y*cv->w+x]=color;
}

I64 RosCanvasGet(CRosCanvas *cv,I64 x,I64 y)
{
  if (x>=0 && y>=0 && x<cv->w && y<cv->h)
    return cv->px[y*cv->w+x];
  return -1;
}

//Nearest palette color to r,g,b (0..255 each).
I64 RosColor16(I64 r,I64 g,I64 b)
{
  I64 i,best=0,d,bd=0x7FFFFFFF,pr,pg,pb;
  for (i=0;i<16;i++) {
    pr=(ros_palette[i]>>16)&255;
    pg=(ros_palette[i]>>8)&255;
    pb=ros_palette[i]&255;
    d=(r-pr)*(r-pr)+(g-pg)*(g-pg)+(b-pb)*(b-pb);
    if (d<bd) {
      bd=d;
      best=i;
    }
  }
  return best;
}

//Line clipped to the canvas (Bresenham). Far-away ends are clipped
//first so huge coordinates don't loop forever.
U0 RosCanvasLine(CRosCanvas *cv,I64 x0,I64 y0,I64 x1,I64 y1,I64 color)
{
  I64 dx,dy,sx,sy,err,e2,n=0;
  if ((x0<0 && x1<0) || (y0<0 && y1<0) || (x0>=cv->w && x1>=cv->w) || (y0>=cv->h && y1>=cv->h))
    return;
  if (AbsI64(x0)>1000000 || AbsI64(y0)>1000000 || AbsI64(x1)>1000000 || AbsI64(y1)>1000000)
    return;	//nowhere near the screen (huge zoom)
  dx=AbsI64(x1-x0);
  dy=-AbsI64(y1-y0);
  if (x0<x1)
    sx=1;
  else
    sx=-1;
  if (y0<y1)
    sy=1;
  else
    sy=-1;
  err=dx+dy;
  while (TRUE) {
    RosCanvasPlot(cv,x0,y0,color);
    if (x0==x1 && y0==y1)
      break;
    e2=2*err;
    if (e2>=dy) {
      err+=dy;
      x0+=sx;
    }
    if (e2<=dx) {
      err+=dx;
      y0+=sy;
    }
    n++;
    if (n>2000000)	//safety net
      break;
  }
}

U0 RosCanvasRect(CRosCanvas *cv,I64 x0,I64 y0,I64 x1,I64 y1,I64 color)	//filled, inclusive
{
  I64 x,y,t;
  if (x1<x0) {
    t=x0;
    x0=x1;
    x1=t;
  }
  if (y1<y0) {
    t=y0;
    y0=y1;
    y1=t;
  }
  if (x0<0)
    x0=0;
  if (y0<0)
    y0=0;
  if (x1>=cv->w)
    x1=cv->w-1;
  if (y1>=cv->h)
    y1=cv->h-1;
  for (y=y0;y<=y1;y++)
    for (x=x0;x<=x1;x++)
      cv->px[y*cv->w+x]=color;
}

U0 RosCanvasCircle(CRosCanvas *cv,I64 cx,I64 cy,I64 r,I64 color,Bool fill)
{
  I64 x,y;
  if (r<=0) {
    RosCanvasPlot(cv,cx,cy,color);
    return;
  }
  if (r>4000)
    r=4000;
  for (y=-r;y<=r;y++)
    for (x=-r;x<=r;x++)
      if (x*x+y*y<=r*r && (fill || x*x+y*y>(r-1)*(r-1)))
        RosCanvasPlot(cv,cx+x,cy+y,color);
}

//The canvas as a binary PPM (P6) image (new buffer, length in *len).
U8 *RosCanvasPpm(CRosCanvas *cv,I64 *len)
{
  U8 *head=MStrPrint("P6\n%d %d\n255\n",cv->w,cv->h),*out,*p;
  I64 n=StrLen(head),i,c;
  *len=n+cv->w*cv->h*3;
  out=MAlloc(*len);
  MemCpy(out,head,n);
  p=out+n;
  for (i=0;i<cv->w*cv->h;i++) {
    c=ros_palette[cv->px[i]&15];
    p[0]=(c>>16)&255;
    p[1]=(c>>8)&255;
    p[2]=c&255;
    p+=3;
  }
  Free(head);
  return out;
}

Bool RosCanvasSavePpm(CRosCanvas *cv,U8 *path)
{
  I64 len;
  U8 *d=RosCanvasPpm(cv,&len);
  Bool ok=RosFileWrite(path,d,len);
  Free(d);
  return ok;
}
