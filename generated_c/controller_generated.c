/* AUTO-GENERATED from auroraarm/controller.py; do not edit manually.
 * Traceability: REQ-CTRL-AA-001..004, REQ-SAFE-AA-001, TEST-SIL-001. */
#include <stdio.h>
static float integral = 0.0f, previous_error = 0.0f;
float controller_step(float reference, float measurement) {
    const float kp=10.0f, ki=0.5f, kd=0.3f, Ts=0.002f, kaw=20.0f;
    const float umin=-48.0f, umax=48.0f;
    float error=reference-measurement;
    float derivative=(error-previous_error)/Ts;
    float unsaturated=kp*error+integral+kd*derivative;
    float u=unsaturated;
    if (u>umax) u=umax; if (u<umin) u=umin;
    integral += Ts*(ki*error+kaw*(u-unsaturated));
    previous_error=error;
    return u;
}
int main(void) {
    float r,y; while (scanf("%f %f", &r, &y)==2) printf("%.9g\n", controller_step(r,y));
    return 0;
}
