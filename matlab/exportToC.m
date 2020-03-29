function exportToC()
    global A B K L
    k = num2str(K);
    k = regexprep(k,'\s+',',');
    ABK = A-B*K;
    abk2 = num2str(ABK(2,:));
    abk2 = regexprep(abk2,'\s+',',');
    abk4 = num2str(ABK(4,:));
    abk4 = regexprep(abk4,'\s+',',');
    l1 = num2str(L(1,:));
    l1 = regexprep(l1,'\s+',',');
    l2 = num2str(L(2,:));
    l2 = regexprep(l2,'\s+',',');
    l3 = num2str(L(3,:));
    l3 = regexprep(l3,'\s+',',');
    l4 = num2str(L(4,:));
    l4 = regexprep(l4,'\s+',',');

    % Save as 'sample.h' file
    fid = fopen('obsv.h','w');
    fprintf(fid,'//Gains for K, A-B*K, L from MATLAB\n');
    fprintf(fid,'float k[4] = {%s};\n',k);
    fprintf(fid,'float abk2[4] = {%s};\n',abk2);
    fprintf(fid,'float abk4[4] = {%s};\n',abk4);
    fprintf(fid,'float l1[2] = {%s};\n',l1);
    fprintf(fid,'float l2[2] = {%s};\n',l2);
    fprintf(fid,'float l3[2] = {%s};\n',l3);
    fprintf(fid,'float l4[2] = {%s};\n',l4);
    fclose(fid);

end

