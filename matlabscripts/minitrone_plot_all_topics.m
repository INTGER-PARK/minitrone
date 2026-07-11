clear; clc; close all;

%========================================================%
% Plot settings
%========================================================%
enable_sync_x = true;

enable_plot_xlim = false;
plot_xlim = [30, 50];
enable_plot_xtick = false;
plot_xtick_step = 10;

enable_external_moment_ylim = false;
external_moment_ylim = [-0.2, 0.2];
enable_external_moment_ytick = false;
external_moment_ytick_step = 0.1;

enable_external_force_ylim = false;
external_force_ylim = [-0.3, 0.3];
enable_external_force_ytick = false;
external_force_ytick_step = 0.1;

impedance_n_face_body = [1, 0, 0];

%========================================================%
% Read CSV

    script_dir = fileparts(mfilename("fullpath"));
    csv_path = fullfile(script_dir, "..", "csv_out", "bag_all_20260619_125206.csv");


assert(isfile(csv_path), "Missing CSV file: %s", csv_path);
fprintf("Reading CSV: %s\n", csv_path);
all_topics = readtable(csv_path);

all_topics = normalize_csv_columns(all_topics);

assert(any(strcmp(string(all_topics.Properties.VariableNames), "topic_name")), ...
    "CSV must contain topic_name or topic column.");
assert(any(strcmp(string(all_topics.Properties.VariableNames), "t_sec")), ...
    "CSV must contain t_sec or time_sec column.");

%========================================================%
% Split topics
%========================================================%
state = topic_table(all_topics, "/minitrone/state");
cmd = topic_table(all_topics, "/minitrone/cmd");
att = topic_table(all_topics, "/minitrone/att_cmd");
wrench_cmd = topic_table(all_topics, "/minitrone/wrench_cmd");
input_cmd = topic_table(all_topics, "/minitrone/input");
actuation_wrench_body = topic_table(all_topics, "/minitrone/actuation_wrench_body");
actuation_force_body = topic_table(all_topics, "/minitrone/actuation_force_body");
mob_observer_input = topic_table(all_topics, "/minitrone/mob_observer_input");
wrench_hat = topic_table(all_topics, "/minitrone/external_wrench_hat");
wrench_hat_second = topic_table(all_topics, "/minitrone/external_wrench_hat_second_order");
external_wrench_cmd = topic_table(all_topics, "/minitrone/external_wrench_cmd");
impedance_des_force_topic = topic_table(all_topics, "/minitrone/impedance_des_force");

t0 = choose_t0(state, all_topics);

%========================================================%
% Position Desired vs Real
%========================================================%
if has_rows(state) && has_rows(cmd)
    ts = state.t_sec - t0;
    tc = cmd.t_sec - t0;

    x_des = previous_interp(tc, col(cmd, ["cmd__pos_cmd_0", "minitrone_cmd__pos_cmd_0"], "pos_cmd_0"), ts);
    y_des = previous_interp(tc, col(cmd, ["cmd__pos_cmd_1", "minitrone_cmd__pos_cmd_1"], "pos_cmd_1"), ts);
    z_des = previous_interp(tc, col(cmd, ["cmd__pos_cmd_2", "minitrone_cmd__pos_cmd_2"], "pos_cmd_2"), ts);

    x_real = col(state, ["minitrone_state__pos_0", "state__pos_0"], "pos_0");
    y_real = col(state, ["minitrone_state__pos_1", "state__pos_1"], "pos_1");
    z_real = col(state, ["minitrone_state__pos_2", "state__pos_2"], "pos_2");

    figure("Name", "Position", "Color", "w");
    sgtitle("Position", "FontSize", 15, "FontWeight", "bold");
    plot_triplet(ts, {x_des, y_des, z_des}, ts, {x_real, y_real, z_real}, ...
        ["x [m]", "y [m]", "z [m]"], "des", "real");
    finalize_figure(gcf, enable_plot_xlim, plot_xlim, enable_plot_xtick, plot_xtick_step, enable_sync_x);
else
    warning("Skipping Position plot because /minitrone/state or /minitrone/cmd is missing.");
end

%========================================================%
% Attitude Desired vs Real
%========================================================%
if has_rows(state)
    ts = state.t_sec - t0;
    roll_real = rad2deg(col(state, ["minitrone_state__rpy_0", "state__rpy_0"], "rpy_0"));
    pitch_real = rad2deg(col(state, ["minitrone_state__rpy_1", "state__rpy_1"], "rpy_1"));
    yaw_real = rad2deg(col(state, ["minitrone_state__rpy_2", "state__rpy_2"], "rpy_2"));

    if has_rows(att)
        ta = att.t_sec - t0;
        roll_des = previous_interp(ta, col(att, ["attitude_cmd__roll_ref", "att_cmd__roll_ref"], "roll_ref"), ts);
        pitch_des = previous_interp(ta, col(att, ["attitude_cmd__pitch_ref", "att_cmd__pitch_ref"], "pitch_ref"), ts);
        yaw_des = previous_interp(ta, col(att, ["attitude_cmd__yaw_ref", "att_cmd__yaw_ref"], "yaw_ref"), ts);
    else
        roll_des = zeros(size(ts));
        pitch_des = zeros(size(ts));
        yaw_des = zeros(size(ts));
        warning("Using zero attitude command because /minitrone/att_cmd is missing.");
    end

    figure("Name", "Attitude", "Color", "w");
    sgtitle("Attitude", "FontSize", 15, "FontWeight", "bold");
    plot_triplet(ts, {roll_des, pitch_des, yaw_des}, ts, {roll_real, pitch_real, yaw_real}, ...
        ["roll [deg]", "pitch [deg]", "yaw [deg]"], "des", "real");
    finalize_figure(gcf, enable_plot_xlim, plot_xlim, enable_plot_xtick, plot_xtick_step, enable_sync_x);
end

%========================================================%
% Servo Command vs Plant Servo State
%========================================================%
if has_rows(state) && has_rows(input_cmd)
    ts = state.t_sec - t0;
    ti = input_cmd.t_sec - t0;

    servo_cmd = cell(1, 4);
    servo_real = cell(1, 4);
    for k = 1:4
        servo_cmd{k} = rad2deg(col(input_cmd, "input__u_" + string(k + 3), "u_" + string(k + 3)));
        servo_real{k} = col(state, "minitrone_state__servo_" + string(k - 1), "servo_" + string(k - 1));
    end

    figure("Name", "Servo Angle", "Color", "w");
    sgtitle("Servo Angle", "FontSize", 15, "FontWeight", "bold");
    for k = 1:4
        subplot(4, 1, k);
        plot(ti, servo_cmd{k}, "b--", "LineWidth", 2.0, "DisplayName", "cmd"); hold on;
        plot(ts, servo_real{k}, "r-", "LineWidth", 2.0, "DisplayName", "real");
        yline(64.0, "k--", "LineWidth", 1.5, "DisplayName", "limit");
        yline(-64.0, "k--", "LineWidth", 1.5, "HandleVisibility", "off");
        grid on; xlabel("time [s]", "FontSize", 15, "FontWeight", "bold");
        ylabel("servo" + string(k) + " [deg]", "FontSize", 15, "FontWeight", "bold");
        legend("Location", "northeast");
    end
    finalize_figure(gcf, enable_plot_xlim, plot_xlim, enable_plot_xtick, plot_xtick_step, enable_sync_x);
else
    warning("Skipping Servo plot because /minitrone/state or /minitrone/input is missing.");
end

%========================================================%
% Allocator Input Command
%========================================================%
if has_rows(input_cmd)
    ti = input_cmd.t_sec - t0;
    u = cell(1, 8);
    for k = 1:8
        u{k} = col(input_cmd, "input__u_" + string(k - 1), "u_" + string(k - 1));
    end

    figure("Name", "Allocator Input Command", "Color", "w");
    sgtitle("Allocator Input Command", "FontSize", 15, "FontWeight", "bold");
    labels = ["omega1", "servo1 [deg]", "omega2", "servo2 [deg]", ...
              "omega3", "servo3 [deg]", "omega4", "servo4 [deg]"];
    for k = 1:8
        subplot(4, 2, k);
        y = u{k};
        if k == 2 || k == 4 || k == 6 || k == 8
            y = rad2deg(y);
        end
        plot(ti, y, "b--", "LineWidth", 2.0);
        grid on; xlabel("time [s]", "FontSize", 15, "FontWeight", "bold");
        ylabel(labels(k), "FontSize", 15, "FontWeight", "bold");
    end
    finalize_figure(gcf, enable_plot_xlim, plot_xlim, enable_plot_xtick, plot_xtick_step, enable_sync_x);
end

%========================================================%
% BLDC Thrust Command
%========================================================%
if has_rows(input_cmd)
    ti = input_cmd.t_sec - t0;
    k_thrust = 0.02;
    thrust_cmd = cell(1, 4);
    for k = 1:4
        omega = col(input_cmd, "input__u_" + string(k - 1), "u_" + string(k - 1));
        thrust_cmd{k} = k_thrust * max(omega, 0).^2;
    end

    figure("Name", "BLDC Thrust Command", "Color", "w");
    sgtitle("BLDC Thrust Command", "FontSize", 15, "FontWeight", "bold");
    for k = 1:4
        subplot(4, 1, k);
        plot(ti, thrust_cmd{k}, "b-", "LineWidth", 2.0);
        yline(15, "k--", "LineWidth", 1.5, "DisplayName", "limit");
        grid on;
        xlabel("time [s]", "FontSize", 15, "FontWeight", "bold");
        ylabel("T" + string(k) + " [N]", "FontSize", 15, "FontWeight", "bold");
        legend("Location", "northeast");
    end
    finalize_figure(gcf, enable_plot_xlim, plot_xlim, enable_plot_xtick, plot_xtick_step, enable_sync_x);
else
    warning("Skipping BLDC Thrust plot because /minitrone/input is missing.");
end

%========================================================%
% Controller Wrench Command
%========================================================%
if has_rows(wrench_cmd)
    tw = wrench_cmd.t_sec - t0;
    [M, F] = wrench_columns(wrench_cmd, "wrench_cmd");

    figure("Name", "Controller Wrench Command", "Color", "w");
    sgtitle("Controller Wrench Command", "FontSize", 15, "FontWeight", "bold");
    plot_wrench_grid(tw, M, F, "b-");
    finalize_figure(gcf, enable_plot_xlim, plot_xlim, enable_plot_xtick, plot_xtick_step, enable_sync_x);
else
    warning("Skipping Controller Wrench Command plot because /minitrone/wrench_cmd is missing.");
end

%========================================================%
% Normal Force
%========================================================%
if has_rows(wrench_hat_second)
    th = wrench_hat_second.t_sec - t0;
    [~, Fhat_imp] = wrench_columns(wrench_hat_second, "external_wrench_hat_second_order");

    n_face_body = impedance_n_face_body(:);
    if norm(n_face_body) < 1e-9
        n_face_body = [1; 0; 0];
    end
    n_face_body = n_face_body / norm(n_face_body);

    f_hat_normal = max(0, -(n_face_body(1) * Fhat_imp{1} + ...
                            n_face_body(2) * Fhat_imp{2} + ...
                            n_face_body(3) * Fhat_imp{3}));

    if has_rows(impedance_des_force_topic)
        td = impedance_des_force_topic.t_sec - t0;
        f_des = col(impedance_des_force_topic, "impedance_des_force__data", "data");
        f_des_normal = previous_interp(td, f_des, th);
        has_f_des_normal = true;
    else
        has_f_des_normal = false;
        warning("No /minitrone/impedance_des_force topic; skipping F_des in Normal Force plot.");
    end

    has_act_normal = has_rows(actuation_wrench_body);
    if has_act_normal
        ta = actuation_wrench_body.t_sec - t0;
        [~, Fact_imp] = wrench_columns(actuation_wrench_body, "actuation_wrench_body");
        f_act_normal = n_face_body(1) * Fact_imp{1} + ...
                       n_face_body(2) * Fact_imp{2} + ...
                       n_face_body(3) * Fact_imp{3};
    end

    figure("Name", "Normal Force", "Color", "w");
    sgtitle("Normal Force", ...
        "FontSize", 15, "FontWeight", "bold");
    if has_f_des_normal
        plot(th, f_des_normal, "b--", "LineWidth", 2.0, ...
            "DisplayName", "F_{des}"); hold on;
    else
        hold on;
    end
    plot(th, f_hat_normal, "r--", "LineWidth", 2.0, ...
        "DisplayName", "\F_hat");
    if has_act_normal
        plot(ta, f_act_normal, "m-", "LineWidth", 1.5, ...
            "DisplayName", "F_{N_{lpf}}");
    end
    grid on;
    xlabel("time [s]", "FontSize", 15, "FontWeight", "bold");
    ylabel("normal force [N]", "FontSize", 15, "FontWeight", "bold");
    legend("Location", "northeast");
    finalize_figure(gcf, enable_plot_xlim, plot_xlim, enable_plot_xtick, plot_xtick_step, enable_sync_x);
else
    warning("Skipping Normal Force plot because /minitrone/external_wrench_hat_second_order is missing.");
end

%========================================================%
% Actual Actuation Body Wrench
%========================================================%
if has_rows(actuation_wrench_body)
    tf = actuation_wrench_body.t_sec - t0;
    [M, F] = wrench_columns(actuation_wrench_body, "actuation_wrench_body");

    figure("Name", "Actual Actuation Body Wrench", "Color", "w");
    sgtitle("Actual Actuation Body Wrench", "FontSize", 15, "FontWeight", "bold");
    plot_wrench_grid(tf, M, F, "r-");
    finalize_figure(gcf, enable_plot_xlim, plot_xlim, enable_plot_xtick, plot_xtick_step, enable_sync_x);
elseif has_rows(actuation_force_body)
    tf = actuation_force_body.t_sec - t0;
    Fx = col(actuation_force_body, ["body_force__force_0", "actuation_force_body__force_0"], "force_0");
    Fy = col(actuation_force_body, ["body_force__force_1", "actuation_force_body__force_1"], "force_1");
    Fz = col(actuation_force_body, ["body_force__force_2", "actuation_force_body__force_2"], "force_2");

    figure("Name", "Actual Actuation Body Force", "Color", "w");
    sgtitle("Actual Actuation Body Force", "FontSize", 15, "FontWeight", "bold");
    plot_scalar_stack(tf, {Fx, Fy, Fz}, ["Fx_B [N]", "Fy_B [N]", "Fz_B [N]"], "r-");
    finalize_figure(gcf, enable_plot_xlim, plot_xlim, enable_plot_xtick, plot_xtick_step, enable_sync_x);
elseif has_rows(mob_observer_input)
    tf = mob_observer_input.t_sec - t0;
    Mx = col(mob_observer_input, "mob_observer_input__actuation_moment_0", "actuation_moment_0");
    My = col(mob_observer_input, "mob_observer_input__actuation_moment_1", "actuation_moment_1");
    Mz = col(mob_observer_input, "mob_observer_input__actuation_moment_2", "actuation_moment_2");
    Fx = col(mob_observer_input, "mob_observer_input__actuation_force_0", "actuation_force_0");
    Fy = col(mob_observer_input, "mob_observer_input__actuation_force_1", "actuation_force_1");
    Fz = col(mob_observer_input, "mob_observer_input__actuation_force_2", "actuation_force_2");

    figure("Name", "Actual Actuation Body Wrench", "Color", "w");
    sgtitle("Actual Actuation Body Wrench", "FontSize", 15, "FontWeight", "bold");
    plot_wrench_grid(tf, {Mx, My, Mz}, {Fx, Fy, Fz}, "r-");
    finalize_figure(gcf, enable_plot_xlim, plot_xlim, enable_plot_xtick, plot_xtick_step, enable_sync_x);
else
    warning("Skipping Actual Actuation Body Wrench plot because actuation wrench data is missing.");
end
%========================================================%
% External Wrench Cmd Only
%========================================================%
if has_rows(external_wrench_cmd)
    te = external_wrench_cmd.t_sec - t0;
    [Mcmd, Fcmd] = wrench_columns(external_wrench_cmd, "external_wrench_cmd");

    figure("Name", "External Wrench Cmd Only", "Color", "w");
    sgtitle("External Wrench Cmd Only", "FontSize", 15, "FontWeight", "bold");

    names = ["Mx_{cmd} [Nm]", "My_{cmd} [Nm]", "Mz_{cmd} [Nm]", ...
             "Fx_{cmd} [N]",  "Fy_{cmd} [N]",  "Fz_{cmd} [N]"];

    for k = 1:6
        subplot(3, 2, subplot_index(k));
        plot(te, wrench_component(Mcmd, Fcmd, k), "b-", "LineWidth", 2.0);
        grid on;
        xlabel("time [s]", "FontSize", 15, "FontWeight", "bold");
        ylabel(names(k), "FontSize", 15, "FontWeight", "bold");

        if k <= 3
            if enable_external_moment_ylim
                ylim(external_moment_ylim);
            end
            if enable_external_moment_ytick
                yticks(external_moment_ylim(1):external_moment_ytick_step:external_moment_ylim(2));
            end
        else
            if enable_external_force_ylim
                ylim(external_force_ylim);
            end
            if enable_external_force_ytick
                yticks(external_force_ylim(1):external_force_ytick_step:external_force_ylim(2));
            end
        end
    end

    finalize_figure(gcf, enable_plot_xlim, plot_xlim, enable_plot_xtick, plot_xtick_step, enable_sync_x);
else
    warning("Skipping External Wrench Cmd Only plot because /minitrone/external_wrench_cmd is missing.");
end
%========================================================%
% External Wrench Cmd vs Estimates
%========================================================%
has_first = has_rows(wrench_hat);
has_second = has_rows(wrench_hat_second);
has_cmd = has_rows(external_wrench_cmd);

if has_first || has_second || has_cmd
    figure("Name", "Applied External Wrench Cmd vs Estimates", "Color", "w");
    sgtitle("Applied External Wrench Cmd vs Estimates", "FontSize", 15, "FontWeight", "bold");
    names = ["Mx", "My", "Mz", "Fx", "Fy", "Fz"];

    for k = 1:6
        subplot(3, 2, subplot_index(k)); hold on;
        if has_cmd
            [Mcmd, Fcmd] = wrench_columns(external_wrench_cmd, "external_wrench_cmd");
            plot(external_wrench_cmd.t_sec - t0, wrench_component(Mcmd, Fcmd, k), ...
                "b--", "LineWidth", 2.0, "DisplayName", "$w_{\mathrm{ext,cmd}}$");
        end
        if has_first
            [Mhat, Fhat] = wrench_columns(wrench_hat, "external_wrench_hat");
            plot(wrench_hat.t_sec - t0, wrench_component(Mhat, Fhat, k), ...
                "r-", "LineWidth", 2.0, "DisplayName", "$\hat{w}_{\mathrm{ext,first}}$");
        end
        if has_second
            [Mhat2, Fhat2] = wrench_columns(wrench_hat_second, "external_wrench_hat_second_order");
            plot(wrench_hat_second.t_sec - t0, wrench_component(Mhat2, Fhat2, k), ...
                "m-", "LineWidth", 2.0, "DisplayName", "$\hat{w}_{\mathrm{ext,second}}$");
        end
        grid on; xlabel("time [s]", "FontSize", 15, "FontWeight", "bold");
        ylabel(names(k), "FontSize", 15, "FontWeight", "bold");
        if k <= 3
            if enable_external_moment_ylim, ylim(external_moment_ylim); end
            if enable_external_moment_ytick
                yticks(external_moment_ylim(1):external_moment_ytick_step:external_moment_ylim(2));
            end
        else
            if enable_external_force_ylim, ylim(external_force_ylim); end
            if enable_external_force_ytick
                yticks(external_force_ylim(1):external_force_ytick_step:external_force_ylim(2));
            end
        end
        legend("show", "Interpreter", "latex", "Location", "northeast", "FontSize", 12);
    end
    finalize_figure(gcf, enable_plot_xlim, plot_xlim, enable_plot_xtick, plot_xtick_step, enable_sync_x);
else
    warning("Skipping External Wrench plot because command/estimate topics are missing.");
end

%========================================================%
% Local functions
%========================================================%
function tbl = topic_table(all_topics, topic)
    tbl = all_topics(strcmp(string(all_topics.topic_name), topic), :);
end

function tbl = normalize_csv_columns(tbl)
    names = string(tbl.Properties.VariableNames);
    if ~any(names == "topic_name") && any(names == "topic")
        tbl.topic_name = tbl.topic;
    end
    names = string(tbl.Properties.VariableNames);
    if ~any(names == "t_sec") && any(names == "time_sec")
        tbl.t_sec = tbl.time_sec;
    end
end

function ok = has_rows(tbl)
    ok = ~isempty(tbl) && height(tbl) > 0;
end

function t0 = choose_t0(state, all_topics)
    if has_rows(state)
        t0 = state.t_sec(1);
    else
        t0 = min(all_topics.t_sec);
    end
end

function y = col(tbl, exact_names, suffix)
    names = string(tbl.Properties.VariableNames);
    exact_names = string(exact_names);
    namespaced_exact_names = "minitrone__" + exact_names;

    for candidate = [exact_names, namespaced_exact_names]
        idx = find(names == candidate, 1);
        if ~isempty(idx)
            y = tbl.(names(idx));
            return;
        end
    end

    suffix = string(suffix);
    candidates = find(endsWith(names, "__" + suffix) | endsWith(names, "_" + suffix));
    if isempty(candidates)
        candidates = find(contains(names, suffix));
    end
    assert(~isempty(candidates), "Missing column ending with '%s'. Available columns include: %s", ...
        suffix, strjoin(names(1:min(end, 20)), ", "));

    for idx = candidates
        values = tbl.(names(idx));
        if any(~ismissing(values))
            y = values;
            return;
        end
    end

    y = tbl.(names(candidates(1)));
end

function yq = previous_interp(t, y, tq)
    if numel(t) < 2
        yq = repmat(y(1), size(tq));
    else
        yq = interp1(t, y, tq, "previous", "extrap");
    end
end

function plot_triplet(t_des, y_des, t_real, y_real, ylabels, des_name, real_name)
    for k = 1:3
        subplot(3, 1, k);
        plot(t_des, y_des{k}, "b--", "LineWidth", 2.0, "DisplayName", des_name); hold on;
        plot(t_real, y_real{k}, "r-", "LineWidth", 2.0, "DisplayName", real_name);
        grid on; xlabel("time [s]", "FontSize", 15, "FontWeight", "bold");
        ylabel(ylabels(k), "FontSize", 15, "FontWeight", "bold");
        legend("Location", "northeast");
    end
end

function plot_scalar_stack(t, y, ylabels, style)
    for k = 1:numel(y)
        subplot(numel(y), 1, k);
        plot(t, y{k}, style, "LineWidth", 2.0);
        grid on; xlabel("time [s]", "FontSize", 15, "FontWeight", "bold");
        ylabel(ylabels(k), "FontSize", 15, "FontWeight", "bold");
    end
end

function [M, F] = wrench_columns(tbl, prefix)
    prefix = string(prefix);
    M = cell(1, 3);
    F = cell(1, 3);
    for k = 1:3
        M{k} = col(tbl, prefix + "__moment_" + string(k - 1), "moment_" + string(k - 1));
        F{k} = col(tbl, prefix + "__force_" + string(k - 1), "force_" + string(k - 1));
    end
end

function plot_wrench_grid(t, M, F, style)
    labels = ["Mx", "My", "Mz", "Fx", "Fy", "Fz"];
    for k = 1:6
        subplot(3, 2, subplot_index(k));
        plot(t, wrench_component(M, F, k), style, "LineWidth", 2.0);
        grid on; xlabel("time [s]", "FontSize", 15, "FontWeight", "bold");
        ylabel(labels(k), "FontSize", 15, "FontWeight", "bold");
    end
end

function y = wrench_component(M, F, k)
    if k <= 3
        y = M{k};
    else
        y = F{k - 3};
    end
end

function idx = subplot_index(k)
    order = [1, 3, 5, 2, 4, 6];
    idx = order(k);
end

function finalize_figure(fig_handle, enable_xlim, x_range, enable_xtick, xtick_step, enable_sync_x)
    axes_handles = findall(fig_handle, "Type", "axes");
    for ax = axes_handles'
        if enable_xlim
            xlim(ax, x_range);
        end
        if enable_xtick
            if enable_xlim
                tick_range = x_range;
            else
                tick_range = xlim(ax);
            end
            xticks(ax, tick_range(1):xtick_step:tick_range(2));
        end
    end
    if enable_sync_x
        linkaxes(axes_handles, "x");
    end
end
