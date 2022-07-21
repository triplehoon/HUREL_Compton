reconPC = pcread("comptonImg.ply");
figure;
pcshow(reconPC);

lm = readmatrix("20220706_DigitalLabScan_100uCi_-1,0,2.4_cpplmdata.csv");
%%
histogram(lm(:,5));
