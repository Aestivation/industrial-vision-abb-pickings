MODULE MainModule
    ! Calibrated tools and work objects
    TASK PERS tooldata tool1:=[TRUE,[[17.5849,-8.47069,208.256],[1,0,0,0]],[1,[1,0,0],[1,0,0,0],0,0,0]];
    TASK PERS wobjdata GRASPING:=[FALSE,TRUE,"",[[659.52,-211.23,426.188],[0.999943,0.000319963,-0.00964373,-0.0046309]],[[0,0,0],[1,0,0,0]]];
    TASK PERS tooldata graspig_tool:=[TRUE,[[91.7657,-45.8465,198.208],[1,0,0,0]],[1.1,[20,20,25],[1,0,0,0],0,0,0]];
    TASK PERS tooldata tool2:=[TRUE,[[-6.77551,-19.0779,206.72],[1,0,0,0]],[1.1,[0,0,2],[1,0,0,0],0,0,0]];

    ! network variables
    VAR socketdev server_socket;
    VAR socketdev client_socket;
    VAR string receive_string;

    ! data from Python
    VAR string sMaterial;
    VAR string sX;
    VAR string sY;
    VAR string sZ;
    VAR string sOrientation;
    VAR num nX;
    VAR num nY;
    VAR num nZ;
    VAR num nOrientation;
    VAR num nPos1;
    VAR num nPos2;
    VAR num nPos3;
    VAR num nPos4;
    VAR bool bOk;

    VAR robtarget pPick;

    ! Home position
    VAR robtarget pHome := [[146.81,177.12,169.77],[0.00197362,-0.972417,0.233075,-0.00880615],[-1,0,2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

    PROC main()
        ConfJ \Off;
        ConfL \Off;
        SingArea \Wrist;

        SetDO doValve2, 0;
        MoveJ pHome, v200, fine, graspig_tool \WObj:=GRASPING;

        ! NOTE: replace with your controller's actual IP and port.
        SocketCreate server_socket;
        SocketBind server_socket, "YOUR_ROBOT_IP", 1025;
        SocketListen server_socket;
        TPWrite "Waiting for AI (Python) to connect...";
        SocketAccept server_socket, client_socket;
        TPWrite "AI Connected! Starting sorting loop.";

        WHILE TRUE DO
            ! 1. Listen for data
            SocketReceive client_socket \Str:=receive_string;

            ! 2. Split the string apart
            nPos1 := StrFind(receive_string, 1, ",");
            nPos2 := StrFind(receive_string, nPos1 + 1, ",");
            nPos3 := StrFind(receive_string, nPos2 + 1, ",");
            nPos4 := StrFind(receive_string, nPos3 + 1, ",");

            sMaterial := StrPart(receive_string, 1, nPos1 - 1);
            sX := StrPart(receive_string, nPos1 + 1, nPos2 - nPos1 - 1);
            sY := StrPart(receive_string, nPos2 + 1, nPos3 - nPos2 - 1);
            sZ := StrPart(receive_string, nPos3 + 1, nPos4- nPos3  - 1);
            sOrientation := StrPart(receive_string, nPos4 + 1, StrLen(receive_string) - nPos4);

            bOk := StrToVal(sX, nX);
            bOk := StrToVal(sY, nY);
            bOk := StrToVal(sZ, nZ);
            bOk := StrToVal(sOrientation, nOrientation);

            TPWrite "Found " + sMaterial + ". Orientation: " + sOrientation + " deg.";

            ! 3. robot target set directly from Python's (XGBoost) data
            pPick := CRobT(\Tool:=graspig_tool \WObj:=GRASPING);
            pPick.trans.x := nX;
            pPick.trans.y := nY;
            pPick.trans.z :=nZ;

            ! Z-axis (orientation)
            pPick :=RelTool(pPick, 0, 0, 0 \Rz:=nOrientation);

            ! 4. Gripper

            SetDO doValve2, 0;

            MoveJ Offs(pPick, 0, 0, 100), v200, fine, graspig_tool \WObj:=GRASPING;
            MoveL pPick, v100, fine, graspig_tool \WObj:=GRASPING;

            SetDO doValve2, 1;
            WaitTime 0.5;

            MoveL Offs(pPick, 0, 0, 100), v200, fine, graspig_tool \WObj:=GRASPING;
            MoveJ Offs(pPick, 0, 300, 100), v200, fine, graspig_tool \WObj:=GRASPING;

            SetDO doValve2, 0;
            WaitTime 0.5;

            MoveJ pHome, v200, fine, graspig_tool \WObj:=GRASPING;

        ENDWHILE
    ENDPROC
ENDMODULE
