exe = "D:\KTH Mechatronics\Degree Project\Unity\Executables\rigid spine executable\SMARCUnityHDRP2.exe";

freqs = [0.2, 0.22, 0.25,0.27,0.3,0.32,0.35,0.37,0.4];
phaseShift = 0;
spineAmp=0;
spineStiff=0;
%spineAmp = [0,1,2,3,4,5,6,7,8,9,10];
%spineStiff = [5.184, 8.64,12.096];
%spineStiff = [5.184,6.048,6.912,7.776, 8.64,9.504,10.36,11.232,12.096];
%freqs=0.2;



%RoboIguana.exe -frequency 0.3 -phaseShift 45 -spineAmp 10 -spineStiff 1500

for f = freqs
    for p = phaseShift
        for A=spineAmp
            for s=spineStiff

                cmd = sprintf('"%s" -frequency %.3f -phaseShift %d -spineAmp %.3f -spineStiff %.3f', exe, f, p,A,s);

                disp("Running: " + cmd)

                system(cmd);   % run Unity executable
            end
        end
    end
end