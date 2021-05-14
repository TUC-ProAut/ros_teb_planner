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


% print info
fprintf('executing startup script for teb-planner ros wrapper\n');

% check if running on older matlab version (before 2020b)
% see also
%   https://de.mathworks.com/matlabcentral/answers/623103#answer_525023
%   https://en.wikipedia.org/wiki/MATLAB#Release_history
using_older_matlab = verLessThan('matlab', '9.9');


% setup paths
local_matlab_path  = fileparts(fileparts(mfilename('fullpath')));
messages_gen_path  = fullfile(local_matlab_path, 'msgs');
if (using_older_matlab)
    messages_load_path = fullfile(messages_gen_path, 'matlab_gen', 'msggen');
else
    messages_load_path = fullfile(messages_gen_path, ...
      'matlab_msg_gen_ros1', 'glnxa64', 'install', 'm');
end

% check python version
if (~using_older_matlab && (pyenv().Version ~= "2.7"))
    fprintf('    We are working with ros1 (kinetic, melodic & noetic).\n');
    fprintf('    Therefore, you need to use python 2.7\n');
    fprintf('      >> pyenv(''Version'', ''python2.7'')\n');
end

% check if messages are build
checked_msgs = [];
if (exist(messages_load_path, 'dir'))
    % load messages
    fprintf('    Loading self build messages\n');
    addpath(messages_load_path);

    % check if messages are loaded correctly
    checked_msgs = rosmsg_check();
    if (isempty(checked_msgs))
        warning('Can''t find rosmsgs for teb_planner_pa :-(');
        if (using_older_matlab)
            fprintf('    Did you update javaclasspath.txt ?\n');
        end
    end
else
    warning('Can''t load teb_planner_pa messages :-(');
    fprintf(['    Did you create the custom messages ?\n', ...
      '      >> rosgenmsg(''', messages_gen_path, ''')\n']);
end

% check if errors occurred
if (~isempty(checked_msgs))
    fprintf('Startup check done :-)\n');
else
    fprintf(['    You might also want to read the ', ...
      'teb_planner_pa_matlab/README.md file.\n']);
end

clear checked_msgs local_matlab_path  messages_gen_path ...
  messages_load_path messages_load_link_path messages_load_default_path ...
  using_older_matlab;
