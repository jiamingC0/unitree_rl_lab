No-head PokerFace ET1 tracking policy with reference COM observations.

Assets:
- `exported/policy.onnx` from `/home/galbot/WorkSpace/mjlab/logs/rsl_rl/et1_tracking/2026-05-21_16-42-48_nohead_hist_sdkpd_foot_mesh_center_com_base_com_forward_pokerface004_8gpu/model_7999.onnx`
- `params/poker_face.npz` from `/home/galbot/WorkSpace/mjlab/single_traj/convert/PokerFace004_et1_1.npz`
```
cp 2026-05-21_15-59-52_nohead_hist_sdkpd_foot_mesh_center_com_high_joint_noise_pokerface004/model_19999.onnx /home/galbot/Deploy/unitree_rl_lab/deploy/robots/et1/config/policy/dance/nohead_poker_face_com/exported/sdkpd_foot_mesh_com_joint.onnx
cp 2026-05-21_16-26-29_nohead_hist_sdkpd_foot_mesh_center_com_robust_pokerface004/model_19999.onnx /home/galbot/Deploy/unitree_rl_lab/deploy/robots/et1/config/policy/dance/nohead_poker_face_com/exported/sdkpd_foot_mesh_com_robust.onnx
cp 2026-05-21_12-03-01_nohead_hist_sdkpd_foot_mesh_center_com_pokerface004_8gpu/model_5999.onnx /home/galbot/Deploy/unitree_rl_lab/deploy/robots/et1/config/policy/dance/nohead_poker_face_com/exported/sdkpd_foot_mesh_com.onnx
cp 2026-05-21_16-42-48_nohead_hist_sdkpd_foot_mesh_center_com_base_com_forward_pokerface004_8gpu/model_7999.onnx /home/galbot/Deploy/unitree_rl_lab/deploy/robots/et1/config/policy/dance/nohead_poker_face_com/exported/sdkpd_foot_mesh_com_forward.onnx
```
Training task: `MjT-NHHF-ET1-SdkPd-FootMesh-CenterCom-8GPU`.
