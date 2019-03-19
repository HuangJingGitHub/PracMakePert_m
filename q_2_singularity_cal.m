function q_2_sity = q_2_singularity_cal(L, W, d0, beta, gamma)
    q_2_sity = atan((L + d0 * cos(beta)) / (W + d0 * sin(beta)) / cos(gamma)) * 180 / pi;
end