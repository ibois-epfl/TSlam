#include <stdio.h>
#include <iostream>
#include <Python.h>


using namespace std;

int main()
{
	char filename[] = "cluster.py";
	FILE* fp;

    int argc = 2;
    const char * argv[2] = {(char*)"/home/tpp/UCOSlam-IBOIS/build/utils/long_new_param_comb.yml", (char*)"2"};  //FIXME: path to update?
    // exported_tag_yml = sys.argv[0]
    // marker_rescale_factor = int(sys.argv[1])

    Py_Initialize();
    
    wchar_t** _argv = (wchar_t**)PyMem_Malloc(sizeof(wchar_t*)*argc);
    for (int i=0; i<argc; i++) {
        wchar_t* arg = Py_DecodeLocale(argv[i], NULL);
        _argv[i] = arg;
    }

    PySys_SetArgv(argc, _argv);
    fp = _Py_fopen(filename, "r");
    PyRun_SimpleFile(fp, filename);
    Py_Finalize();
    return 0;
}

// g++ run_python.cpp -I/usr/include/python3.9 -lpython3.9
// or with conda: g++ run_python.cpp -I /home/as/anaconda3/envs/aiac/include/python3.9