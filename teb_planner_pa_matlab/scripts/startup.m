%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                             %
% startup.m                                                                   %
% =========                                                                   %
%                                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                             %
% Repository:                                                                 %
%   https://github.com/TUC-ProAut/ros_teb_planner                             %
%                                                                             %
% Chair of Automation Technology, Technische Universität Chemnitz             %
%   https://www.tu-chemnitz.de/etit/proaut                                    %
%                                                                             %
% Authors:                                                                    %
%   Bhakti Danve, Peter Weissig                                               %
%                                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                             %
% New BSD License                                                             %
%                                                                             %
% Copyright (c) 2019-2021 TU Chemnitz                                         %
% All rights reserved.                                                        %
%                                                                             %
% Redistribution and use in source and binary forms, with or without          %
% modification, are permitted provided that the following conditions are met: %
%    * Redistributions of source code must retain the above copyright notice, %
%      this list of conditions and the following disclaimer.                  %
%    * Redistributions in binary form must reproduce the above copyright      %
%      notice, this list of conditions and the following disclaimer in the    %
%      documentation and/or other materials provided with the distribution.   %
%    * Neither the name of the copyright holder nor the names of its          %
%      contributors may be used to endorse or promote products derived from   %
%      this software without specific prior written permission.               %
%                                                                             %
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS         %
% "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   %
% TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR  %
% PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR           %
% CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,       %
% EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,         %
% PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; %
% OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,    %
% WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR     %
% OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF      %
% ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                  %
%                                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                             %
% For usage instructions on how to create the ros custom messages see the     %
%   README.md file.                                                           %
%                                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


fprintf('executing startup script for ros teb-planner\n');

local_matlab_path = [fileparts(fileparts(mfilename('fullpath'))), '/'];
messages_gen_path = [local_matlab_path, 'msgs/'];
messages_load_path = [messages_gen_path, 'matlab_gen/msggen/'];

% check if messages are build
if (exist(messages_load_path, 'dir'))
    % load messages
    addpath(messages_load_path);
else
    fprintf('    Can''t load teb_planner_pa messages :-(\n');
    fprintf('    Did you create the custom messages ?\n');
    fprintf(['      >> rosgenmsg(''', messages_gen_path, ''')\n']);
    warning(['Steps mentioned in teb_planner_pa_matlab/Readme.md file ',...
             'might be missed or not correctly followed...']);

    return
end
clear local_matlab_path  messages_gen_path messages_load_path;

% check if messages are loaded (including java-class)
temp = rosmsg_check();
if (isempty(temp))
    fprintf('    Can''t find rosmsgs for teb-planner :-(\n');
    fprintf('    Did you update javaclasspath.txt ?\n');
else
    fprintf('Startup check done.\n');
end
clear temp;
