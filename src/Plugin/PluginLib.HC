//PluginLib: the ROS2-Holy-C replacement for pluginlib.
//
//A plugin is a HolyC source file. When it is loaded (PluginLoad)
//its top-level code runs and registers its classes and methods:
//
//  PluginClassAdd("polygon_plugins::Square","polygon_base::RegularPolygon",
//	&Square_New,&Square_Del);
//  PluginMethodAdd("polygon_plugins::Square","area",&Square_area);
//
//Users then do:
//
//  CPluginObj *o=PluginCreate("polygon_plugins::Square");
//  F64 (*area)(U8 *self)=PluginMethod(o,"area");
//  Print("%f\n",(*area)(o->self));
//  PluginDestroy(o);
//
//Because plugins are source, PluginReload() can swap in new code
//while everything keeps running. Registering a class or method a
//second time just replaces the old function pointers.

class CPluginMethod
{
  CPluginMethod *next;
  U8 *name;
  U8 *fp;
};

class CPluginClass
{
  CPluginClass *next;
  U8 *name,*base,*desc,*file;
  U8 *(*ctor)();
  U0 (*dtor)(U8 *self);
  CPluginMethod *methods;
  I64 live_objs;
};

class CPluginObj
{
  CPluginClass *cls;
  U8 *self;
};

CPluginClass *plugin_classes=NULL;
U8 *plugin_cur_file=NULL;

//Find a class by its full name ("pkg::Class"), NULL if unknown.
CPluginClass *PluginClassFind(U8 *name)
{
  CPluginClass *c=plugin_classes;
  while (c) {
    if (!StrCmp(c->name,name))
      return c;
    c=c->next;
  }
  return NULL;
}

//Find or create the entry for a class name.
CPluginClass *PluginClassGet(U8 *name)
{
  CPluginClass *c=PluginClassFind(name);
  if (!c) {
    c=CAlloc(sizeof(CPluginClass));
    c->name=StrNew(name);
    c->next=plugin_classes;
    plugin_classes=c;
  }
  return c;
}

//Called from a plugins.xml manifest (cpp2hc manifest output).
U0 PluginDescribe(U8 *name,U8 *base,U8 *desc)
{
  CPluginClass *c=PluginClassGet(name);
  Free(c->base); c->base=StrNew(base);
  Free(c->desc); c->desc=StrNew(desc);
}

//Called from a plugin's top-level code when it is loaded.
U0 PluginClassAdd(U8 *name,U8 *base,U8 *ctor,U8 *dtor)
{
  CPluginClass *c=PluginClassGet(name);
  Free(c->base); c->base=StrNew(base);
  if (plugin_cur_file) {
    Free(c->file);
    c->file=StrNew(plugin_cur_file);
  }
  c->ctor=ctor;
  c->dtor=dtor;
}

//Register (or replace) one method of a plugin class.
U0 PluginMethodAdd(U8 *cls_name,U8 *method,U8 *fp)
{
  CPluginClass *c=PluginClassGet(cls_name);
  CPluginMethod *m=c->methods;
  while (m) {
    if (!StrCmp(m->name,method)) {
      m->fp=fp;
      return;
    }
    m=m->next;
  }
  m=CAlloc(sizeof(CPluginMethod));
  m->name=StrNew(method);
  m->fp=fp;
  m->next=c->methods;
  c->methods=m;
}

//JIT-compile a plugin source file. Its registration code runs now.
U0 PluginLoad(U8 *filename)
{
  plugin_cur_file=filename;
//@host-skip-begin
  ExeFile(filename);
//@host-skip-end
  plugin_cur_file=NULL;
}

//Recompile a plugin file. Existing objects keep their data; new
//method lookups get the new code.
U0 PluginReload(U8 *filename)
{
  PluginLoad(filename);
}

//Make an instance of a registered class. NULL if unknown/not loaded.
CPluginObj *PluginCreate(U8 *name)
{
  CPluginClass *c=PluginClassFind(name);
  CPluginObj *o;
  if (!c || !c->ctor) {
    Print("PluginCreate: class \"%s\" is not loaded\n",name);
    return NULL;
  }
  o=CAlloc(sizeof(CPluginObj));
  o->cls=c;
  o->self=(*c->ctor)();
  c->live_objs++;
  return o;
}

//Get a method's function pointer. Cast it to the right type at the
//call site. NULL if the class has no such method.
U8 *PluginMethod(CPluginObj *o,U8 *method)
{
  CPluginMethod *m;
  if (!o) return NULL;
  m=o->cls->methods;
  while (m) {
    if (!StrCmp(m->name,method))
      return m->fp;
    m=m->next;
  }
  Print("PluginMethod: \"%s\" has no method \"%s\"\n",o->cls->name,method);
  return NULL;
}

U0 PluginDestroy(CPluginObj *o)
{
  if (!o) return;
  if (o->cls->dtor)
    (*o->cls->dtor)(o->self);
  o->cls->live_objs--;
  Free(o);
}

//True if class "name" says it implements "base".
Bool PluginIsA(U8 *name,U8 *base)
{
  CPluginClass *c=PluginClassFind(name);
  return c && c->base && !StrCmp(c->base,base);
}

//List every declared or loaded plugin class.
U0 PluginList()
{
  CPluginClass *c=plugin_classes;
  CPluginMethod *m;
  while (c) {
    Print("%s : %s",c->name,c->base);
    if (c->ctor)
      Print("  [loaded from %s, %d live]\n",c->file,c->live_objs);
    else
      Print("  [declared, not loaded]\n");
    if (c->desc)
      Print("    %s\n",c->desc);
    m=c->methods;
    while (m) {
      Print("    .%s()\n",m->name);
      m=m->next;
    }
    c=c->next;
  }
}
