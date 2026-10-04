executable('minimal', 'dummy/minimal.cpp', dependencies: [cobra_dep])
foreach app : ['pos_ins', 'pos_vel_ins', 'posvel_ins', 'pos_ins_leverarm', 'pos_ins_bodyvel', 'outage_sim', 'pos_ins_vsb', 'direction_to_points', 'pos_ins_record_states']
  executable(app, 'standard/' + app + '.cpp', dependencies: [cobra_dep])
endforeach
executable('tutorial_pos_ins', 'tutorial/pos_ins.cpp', dependencies: [cobra_dep])
executable('tutorial_pos_vel_ins', 'tutorial/pos_vel_ins.cpp', dependencies: [cobra_dep])
executable('pos_ins_zerovel2d', 'extras/pos_ins_zerovel2d.cpp', dependencies: [cobra_dep])
executable('cobra_run', 'cobra_run.cpp', dependencies: [cobra_dep])
