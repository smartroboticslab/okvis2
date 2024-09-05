function [ output_args ] = ... 
    write_statistics_table(destination, statistics )
%WRITE_STATISTICS_TABLE Summary of this function goes here
%   Detailed explanation goes here
hline = '\\hline\n';
newpage = '\\newpage\n';
%open the file
file_id = fopen(destination, 'w');
header_file = '\\documentclass{article} \n \\usepackage{geometry}\n\\usepackage{pdflscape}\n\\usepackage{multirow}\n\\begin{document}';
fprintf(file_id, header_file);



string = '%% complete tables\n';
fprintf(file_id, string);

%% pose statistics
% write the header for the pose error
string = '\\begin{landscape}\n \\begin{center}\n \\begin{tabular}{||*{8}{|c|}||}';
fprintf(file_id, string);

% write column names pose error
fprintf(file_id,hline);
string = '\\multirow{2}{*}{Dataset Name} & OKVIS Version & \\multicolumn{6}{| c |}{ATE [m]} \\\\ \\cline{3-8}\n';
fprintf(file_id, string);
string = '& &  Mean & RMS & Median & $\\sigma$ & Min & Max  \\\\';
fprintf(file_id, string);
fprintf(file_id,hline);
fprintf(file_id,hline);

% write content
for i=1:size(statistics,1)
    name = statistics{i,1};
    name = strrep(name, '_','\_');
    versions = statistics{i,2};
    for k=1:size(versions,2)
        ate_stats = statistics{i,4}{k,1}{1};
        version_name = strrep(versions{k}, '_','\_');
        if k == 1
            string = '\\multirow{2}{*}{%s} & %s & %f &  %f &  %f &%f & %f & %f \\\\ \n';
            fprintf(file_id, string, name, version_name,...
                ate_stats(1),...
                ate_stats(2),...
                ate_stats(3),...
                ate_stats(4),...
                ate_stats(5),...
                ate_stats(6));
        else
            string = ' & %s & %f &  %f &  %f &%f & %f & %f \\\\ \n';
            fprintf(file_id, string, version_name,...
                ate_stats(1),...
                ate_stats(2),...
                ate_stats(3),...
                ate_stats(4),...
                ate_stats(5),...
                ate_stats(6));
        end


    end
end
fprintf(file_id,hline);
fprintf(file_id,hline);

%write the footer 
string = '\n\\end{tabular}\n\\end{center}\n\\end{landscape}\n ';
fprintf(file_id, string);


fprintf(file_id, newpage);

%% orientation statistics
% write the header for the pose error
string = '\\begin{landscape}\n \\begin{center}\n \\begin{tabular}{||*{8}{|c|}||}';
fprintf(file_id, string);

% write column names pose error
fprintf(file_id,hline);
string = '\\multirow{2}{*}{Dataset Name} & OKVIS Version & \\multicolumn{6}{| c |}{Orientation Error [deg]} \\\\ \\cline{3-8}\n';
fprintf(file_id, string);
string = '& &  Mean & RMS & Median & $\\sigma$ & Min & Max  \\\\';
fprintf(file_id, string);
fprintf(file_id,hline);
fprintf(file_id,hline);

% write content
for i=1:size(statistics,1)
    name = statistics{i,1};
    name = strrep(name, '_','\_');
    versions = statistics{i,2};
    for k=1:size(versions,2)
        ote_stats = statistics{i,4}{k,2}{1};
        version_name = strrep(versions{k}, '_','\_');
        if k == 1
            string = '\\multirow{2}{*}{%s} & %s & %f &  %f &  %f &%f & %f & %f \\\\ \n';
            fprintf(file_id, string, name, version_name,...
                ote_stats(1),...
                ote_stats(2),...
                ote_stats(3),...
                ote_stats(4),...
                ote_stats(5),...
                ote_stats(6));
        else
            string = ' & %s & %f &  %f &  %f &%f & %f & %f \\\\ \n';
            fprintf(file_id, string, version_name,...
                ote_stats(1),...
                ote_stats(2),...
                ote_stats(3),...
                ote_stats(4),...
                ote_stats(5),...
                ote_stats(6));
        end


    end
end
fprintf(file_id,hline);
fprintf(file_id,hline);
%write the footer 
string = '\n\\end{tabular}\n\\end{center}\n\\end{landscape}\n ';
fprintf(file_id, string);


fprintf(file_id, newpage);


%% ori z statistics
% write the header for the pose error
string = '\\begin{landscape}\n \\begin{center}\n \\begin{tabular}{||*{8}{|c|}||}';
fprintf(file_id, string);

% write column names pose error
fprintf(file_id,hline);
string = '\\multirow{2}{*}{Dataset Name} & OKVIS Version & \\multicolumn{6}{| c |}{Z Axis [deg]} \\\\ \\cline{3-8}\n';
fprintf(file_id, string);
string = '& &  Mean & RMS & Median & $\\sigma$ & Min & Max  \\\\';
fprintf(file_id, string);
fprintf(file_id,hline);
fprintf(file_id,hline);

% write content
for i=1:size(statistics,1)
    name = statistics{i,1};
    name = strrep(name, '_','\_');
    versions = statistics{i,2};
    for k=1:size(versions,2)
        z_ori_stats = statistics{i,4}{k,3}{1};
        version_name = strrep(versions{k}, '_','\_');
        if k == 1
            string = '\\multirow{2}{*}{%s} & %s & %f &  %f &  %f &%f & %f & %f \\\\ \n';
            fprintf(file_id, string, name, version_name,...
                z_ori_stats(1),...
                z_ori_stats(2),...
                z_ori_stats(3),...
                z_ori_stats(4),...
                z_ori_stats(5),...
                z_ori_stats(6));
        else
            string = ' & %s & %f &  %f &  %f &%f & %f & %f \\\\ \n';
            fprintf(file_id, string, version_name,...
                z_ori_stats(1),...
                z_ori_stats(2),...
                z_ori_stats(3),...
                z_ori_stats(4),...
                z_ori_stats(5),...
                z_ori_stats(6));
        end


    end
end
fprintf(file_id,hline);
fprintf(file_id,hline);

%write the footer 
string = '\n\\end{tabular}\n\\end{center}\n\\end{landscape}\n ';
fprintf(file_id, string);


fprintf(file_id, newpage);

%% small table
% write the header for the pose error
string = '\\begin{landscape}\n \\begin{center}\n \\begin{tabular}{||*{8}{|c|}||}';
fprintf(file_id, string);

% write column names pose error
fprintf(file_id,hline);
string = '\\multirow{2}{*}{Dataset Name} & OKVIS Version & \\multicolumn{2}{| c |}{ATE [m]}  & \\multicolumn{2}{| c |}{Orientation Error [deg]} & \\multicolumn{2}{| c |}{Z [deg]} \\\\ \\cline{3-8}\n';
fprintf(file_id, string);
string = '& &  Mean &  $\\sigma$ &  Mean &  $\\sigma$ &  Mean &  $\\sigma$  \\\\';
fprintf(file_id, string);
fprintf(file_id,hline);
fprintf(file_id,hline);
% write content
for i=1:size(statistics,1)
    name = statistics{i,1};
    name = strrep(name, '_','\_');
    versions = statistics{i,2};
    for k=1:size(versions,2)
        version_name = strrep(versions{k}, '_','\_');
        if k == 1
            string = '\\multirow{2}{*}{%s} & %s & %f &  %f &  %f &%f & %f & %f \\\\ \n';
            fprintf(file_id, string, name, version_name,...
                statistics{i,4}{k,1}{1}(1),...
                statistics{i,4}{k,1}{1}(4),...
                statistics{i,4}{k,2}{1}(1),...
                statistics{i,4}{k,2}{1}(4),...
                statistics{i,4}{k,3}{1}(1),...
                statistics{i,4}{k,3}{1}(4));
        else
            string = ' & %s & %f &  %f &  %f &%f & %f & %f \\\\ \n';
            fprintf(file_id, string, version_name,...
                statistics{i,4}{k,1}{1}(1),...
                statistics{i,4}{k,1}{1}(4),...
                statistics{i,4}{k,2}{1}(1),...
                statistics{i,4}{k,2}{1}(4),...
                statistics{i,4}{k,3}{1}(1),...
                statistics{i,4}{k,3}{1}(4));
        end
        
    end
end
fprintf(file_id,hline);
fprintf(file_id,hline);

%write the footer 
string = '\n\\end{tabular}\n\\end{center}\n\\end{landscape}\n ';
fprintf(file_id, string);

%% close 
fprintf(file_id, '\\end{document}');
fclose(file_id);
end

