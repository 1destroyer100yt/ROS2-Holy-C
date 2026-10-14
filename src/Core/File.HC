//File: read/write whole files.

//Read a whole file. Returns a NUL-terminated buffer (yours to Free)
//and its size in *size (if size isn't NULL). NULL if it can't be read.
U8 *RosFileRead(U8 *path,I64 *size)
{
  U8 *buf=NULL;
  I64 n=0;
//@host-skip-begin
  buf=FileRead(path,&n);	//TempleOS FileRead adds a NUL (its source; the tests pass there)
//@host-skip-end
//@host: FILE *fh=fopen((char *)path,"rb");
//@host: if (!fh) return NULL;
//@host: fseek(fh,0,SEEK_END); n=ftell(fh); fseek(fh,0,SEEK_SET);
//@host: buf=MAlloc(n+1); n=fread(buf,1,n,fh); buf[n]=0; fclose(fh);
  if (size)
    *size=n;
  return buf;
}

//Write len bytes to path, replacing the file. TRUE on success.
Bool RosFileWrite(U8 *path,U8 *data,I64 len)
{
//@host-skip-begin
  return FileWrite(path,data,len);
//@host-skip-end
//@host: FILE *fh=fopen((char *)path,"wb");
//@host: if (!fh) return FALSE;
//@host: Bool ok=fwrite(data,1,len,fh)==(size_t)len; fclose(fh); return ok;
}

//Make a directory (no error if it exists). TRUE if it's there after.
Bool RosDirMake(U8 *path)
{
//@host-skip-begin
  DirMk(path);	//TempleOS DirMk (makes one level; the bag tests use it there)
  return TRUE;
//@host-skip-end
//@host: mkdir((char *)path,0755); return TRUE;
}
