executable('minimal', 'dummy/minimal.cpp', dependencies: [cobra_dep])
foreach app : ['pos_ins', 'pos_vel_ins', 'posvel_ins', 'pos_ins_leverarm', 'pos_ins_bodyvel', 'outage_sim', 'pos_ins_vsb', 'direction_to_points']
  executable(app, 'standard/' + app + '.cpp', dependencies: [cobra_dep])
endforeach
