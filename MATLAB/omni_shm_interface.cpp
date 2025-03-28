#include "mex.h"
#include "matrix.h"
#include "omni_shm_interface.hpp"
#include <sys/shm.h>
#include <cstring>

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
    if (nrhs < 2) {
        mexErrMsgTxt("Usage: shm_interface(key, 'read' or 'write', [data])");
        return;
    }

    int shm_key = static_cast<int>(mxGetScalar(prhs[0]));
    char action[20];
    mxGetString(prhs[1], action, sizeof(action));

    int shm_id = shmget(shm_key, sizeof(SHMMessage_t), 0666);
    if (shm_id == -1)
        mexErrMsgTxt("shmget failed.");

    SHMMessage_t *shm_ptr = (SHMMessage_t*)shmat(shm_id, NULL, 0);
    if (shm_ptr == (void*)-1)
        mexErrMsgTxt("shmat failed.");

    else if (strcmp(action, "read transform") == 0) {
        plhs[0] = mxCreateDoubleMatrix(4, 4, mxREAL);
        double *out = mxGetPr(plhs[0]);
        for(int k = 0; k<16; k++){
            out[k] = shm_ptr->read.omnistate.transform[k];
        }
    }

    else if (strcmp(action, "read position") == 0) {
        plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL);
        double *out = mxGetPr(plhs[0]);
        out[0] = shm_ptr->read.omnistate.position.x;
        out[1] = shm_ptr->read.omnistate.position.y;
        out[2] = shm_ptr->read.omnistate.position.z;
    }

    else if (strcmp(action, "read joint") == 0) {
        plhs[0] = mxCreateDoubleMatrix(6, 1, mxREAL);
        double *out = mxGetPr(plhs[0]);
        for(int k = 0; k<6; k++){
            out[k] = shm_ptr->read.omnistate.thetas[k];
        }
    }

    else if (strcmp(action, "read button") == 0) {
        plhs[0] = mxCreateDoubleMatrix(2, 1, mxREAL);
        double *out = mxGetPr(plhs[0]);
        out[0] = shm_ptr->read.omnistate.buttons[0];
        out[1] = shm_ptr->read.omnistate.buttons[1];
    }


    else if (strcmp(action, "write force") == 0) {
        if (nrhs < 3)
            mexErrMsgTxt("Data required for write action.");

        double *in = mxGetPr(prhs[2]);
        shm_ptr->write.omnifeedback.force.x = in[0];
        shm_ptr->write.omnifeedback.force.y = in[1];
        shm_ptr->write.omnifeedback.force.z = in[2];
    }
    else {
        mexErrMsgTxt("Invalid action specified.");
    }

    shmdt(shm_ptr);
}
