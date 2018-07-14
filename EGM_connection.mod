MODULE EGM_connection

VAR egmident egmID1;

VAR egmstate egmSt1;

CONST egm_minmax egm_minmax_lin1:=[-1,1];
CONST egm_minmax egm_minmax_rot1:=[-2,2];
CONST egm_minmax egm_minmax_joint:=[0,0];

!CONST jointtarget jpos10:=[[0,20,10,0,60,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

CONST jointtarget jpos10:=[[0,0,30,0,60,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
VAR pose corr_frame_offs:=[[0,0,0],[1,0,0,0]];


PROC main()

! Move to start position. Fine point is demanded.

MoveAbsJ jpos10\NoEOffs, v100, fine, tool0;
MoveAbsJ jpos10\NoEOffs, v1000, fine, tool0;

testuc;

ENDPROC


PROC testuc()

EGMReset egmID1;

EGMGetId egmID1;

egmSt1:=EGMGetState(egmID1);
TPWrite "EGM state: "\Num:=egmSt1;


IF egmSt1 <= EGM_STATE_CONNECTED THEN

EGMSetupUC ROB_1, egmID1, "default", "udp_connection_name"\Joint ,\CommTimeout:=1000;
!EGMSetupUC ROB_1, egmID1, "default", "udp_connection_name"\Pose ,\CommTimeout:=1000;

ENDIF

egmSt1:=EGMGetState(egmID1);

TPWrite "EGM state: "\Num:=egmSt1;

!EGMActPose egmID1\Tool:=tool0, corr_frame_offs, EGM_FRAME_WORLD, tool0.tframe, EGM_FRAME_TOOL \x:=egm_minmax_lin1 \y:=egm_minmax_lin1 \z:=egm_minmax_lin1 \rx:=egm_minmax_rot1 \ry:=egm_minmax_rot1 \rz:=egm_minmax_rot1 \LpFilter:=5 \SampleRate:=4 \MaxPosDeviation:=1000 \MaxSpeedDeviation:=100 ;
!EGMRunPose egmID1, EGM_STOP_HOLD \x \y \z \Rx \Ry \Rz \CondTime:=10000 \PosCorrGain:=20;

EGMActJoint egmID1 \J1:=egm_minmax_joint \J2:=egm_minmax_joint \J3:=egm_minmax_joint \J4:=egm_minmax_joint \J5:=egm_minmax_joint \J6:=egm_minmax_joint \SampleRate:=4 \MaxPosDeviation:=1000 \MaxSpeedDeviation:=1000 ;
EGMRunJoint egmID1, EGM_STOP_HOLD \J1 \J2 \J3 \J4 \J5 \J6 \CondTime:=10000 \PosCorrGain:=1.1;


egmSt1:=EGMGetState(egmID1);

IF egmSt1 = EGM_STATE_CONNECTED THEN

TPWrite "Reset EGM instance egmID1";

EGMReset egmID1;

ENDIF

ENDPROC

ENDMODULE
