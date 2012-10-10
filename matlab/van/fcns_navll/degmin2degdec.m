function degdec = degmin2degdec(deg,min);

degdec = sign(deg) * (abs(deg) + min/60);
