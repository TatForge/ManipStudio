import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

// Motion commands: MoveJ (joint angles), MoveL (straight TCP line) and
// MoveC (arc through a middle point); each can also go into the program.
Card {
    id: panel
    title: qsTr("Motion")

    property int mode: 0                                // 0 MoveJ, 1 MoveL, 2 MoveC
    property var joints: [20, -40, 60, 10, 45, -30]     // deg
    property var pose: [0.35, 0.2, 0.3, 180, 0, 90]     // m, deg
    property var mid: [0.3, 0.25, 0.35]                 // m
    property real jointSpeed: 60                        // deg/s
    property real jointAccel: 120                       // deg/s^2
    property real lineSpeed: 0.1                        // m/s
    property real lineAccel: 0.5                        // m/s^2

    readonly property var poseLabels: ["x, " + qsTr("m"), "y, " + qsTr("m"), "z, " + qsTr("m"), "Roll, °", "Pitch, °", "Yaw, °"]

    function setAt(list, i, v) { const l = list.slice(); l[i] = v; return l }
    function rounded(v, digits) { const k = Math.pow(10, digits); return Math.round(v * k) / k }

    function takeCurrent() {
        if (mode === 0)
            joints = robot.joints.map(v => rounded(v, 2))
        else if (robot.tcp.length === 6)
            pose = robot.tcp.map((v, i) => rounded(v, i < 3 ? 4 : 2))
    }
    function step() {
        if (mode === 0)
            return { type: "J", joints: joints, speed: jointSpeed, accel: jointAccel }
        if (mode === 1)
            return { type: "L", pose: pose, speed: lineSpeed, accel: lineAccel }
        return { type: "C", mid: mid, pose: pose, speed: lineSpeed, accel: lineAccel }
    }
    function run() {
        if (mode === 0)
            robot.moveJ(joints, jointSpeed, jointAccel)
        else if (mode === 1)
            robot.moveL(pose, lineSpeed, lineAccel)
        else
            robot.moveC(mid, pose, lineSpeed, lineAccel)
    }

    RowLayout {
        spacing: 6
        Repeater {
            model: ["MoveJ", "MoveL", "MoveC"]
            delegate: Chip {
                required property var modelData
                required property int index
                text: modelData
                checked: panel.mode === index
                onClicked: panel.mode = index
            }
        }
    }

    // MoveJ: target joint angles
    GridLayout {
        visible: panel.mode === 0
        columns: 3
        columnSpacing: 8
        rowSpacing: 2
        Layout.fillWidth: true
        Repeater {
            model: 6
            delegate: RowLayout {
                required property int index
                spacing: 4
                Text { text: "q" + (index + 1); color: Theme.textDim; font.family: Theme.mono }
                NumberField {
                    implicitWidth: 64
                    value: panel.joints[index]
                    onEdited: v => panel.joints = panel.setAt(panel.joints, index, v)
                }
            }
        }
    }

    // MoveC: the point the arc passes through
    Text { visible: panel.mode === 2; text: qsTr("Through point"); color: Theme.textDim; font.pixelSize: 12 }
    GridLayout {
        visible: panel.mode === 2
        columns: 3
        columnSpacing: 8
        Layout.fillWidth: true
        Repeater {
            model: 3
            delegate: RowLayout {
                required property int index
                spacing: 4
                Text { text: ["x", "y", "z"][index]; color: Theme.textDim; font.family: Theme.mono }
                NumberField {
                    implicitWidth: 72
                    value: panel.mid[index]
                    onEdited: v => panel.mid = panel.setAt(panel.mid, index, v)
                }
            }
        }
    }

    // MoveL / MoveC: target pose
    Text { visible: panel.mode === 2; text: qsTr("End pose"); color: Theme.textDim; font.pixelSize: 12 }
    GridLayout {
        visible: panel.mode !== 0
        columns: 3
        columnSpacing: 8
        rowSpacing: 2
        Layout.fillWidth: true
        Repeater {
            model: 6
            delegate: ColumnLayout {
                required property int index
                spacing: 0
                Text { text: panel.poseLabels[index]; color: Theme.textDim; font.pixelSize: 11 }
                NumberField {
                    implicitWidth: 92
                    value: panel.pose[index]
                    onEdited: v => panel.pose = panel.setAt(panel.pose, index, v)
                }
            }
        }
    }

    RowLayout {
        Layout.fillWidth: true
        Text {
            text: panel.mode === 0 ? qsTr("Speed, °/s") : qsTr("Speed, m/s")
            color: Theme.textDim
            Layout.fillWidth: true
        }
        NumberField {
            value: panel.mode === 0 ? panel.jointSpeed : panel.lineSpeed
            onEdited: v => { if (panel.mode === 0) panel.jointSpeed = v; else panel.lineSpeed = v }
        }
    }
    RowLayout {
        Layout.fillWidth: true
        Text {
            text: panel.mode === 0 ? qsTr("Acceleration, °/s²") : qsTr("Acceleration, m/s²")
            color: Theme.textDim
            Layout.fillWidth: true
        }
        NumberField {
            value: panel.mode === 0 ? panel.jointAccel : panel.lineAccel
            onEdited: v => { if (panel.mode === 0) panel.jointAccel = v; else panel.lineAccel = v }
        }
    }

    RowLayout {
        Layout.fillWidth: true
        Button {
            text: qsTr("Current")
            flat: true
            enabled: robot.ready
            onClicked: panel.takeCurrent()
        }
        Button {
            text: qsTr("To program")
            flat: true
            enabled: robot.ready
            onClicked: robot.addStep(panel.step())
        }
        Item { Layout.fillWidth: true }
        Button {
            text: robot.playing ? qsTr("Stop") : qsTr("Run")
            highlighted: true
            enabled: robot.ready && !robot.busy
            onClicked: robot.playing ? robot.stop() : panel.run()
        }
    }
}
