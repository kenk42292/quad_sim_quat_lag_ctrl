function [q0, q1, q2, q3] = euler2quaternion(phi, theta, psi)
    q0 = cos(psi/2)*cos(theta/2)*cos(phi/2)-sin(psi/2)*cos(theta/2)*sin(phi/2);
    q1 = cos(psi/2)*sin(theta/2)*cos(phi/2)+sin(psi/2)*sin(theta/2)*sin(phi/2);
    q2 = sin(psi/2)*sin(theta/2)*cos(phi/2)-cos(psi/2)*sin(theta/2)*sin(phi/2);
    q3 = sin(psi/2)*cos(theta/2)*cos(phi/2)+cos(psi/2)*cos(theta/2)*sin(phi/2);