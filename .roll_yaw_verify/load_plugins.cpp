#include <dlfcn.h>
#include <iostream>
int main(int argc,char** argv)
{
  for(int i=1;i<argc;++i) {
    if(!dlopen(argv[i],RTLD_NOW|RTLD_GLOBAL)) { std::cerr<<dlerror()<<'\n'; return 1; }
    std::cout<<"Loaded: "<<argv[i]<<'\n';
  }
}
