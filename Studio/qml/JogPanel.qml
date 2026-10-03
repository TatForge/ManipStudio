import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

// Moves the tool in short straight steps along the world axes and turns it
// about them (roll / pitch / yaw); each press plans one small MoveL.
Card {
    id: panel
    title: qsTr("Tool jog")

    property real linearStep: 0.01      // m
    property real angularStep: 5        // deg

    readonly property var axes: [
        { label: "X", axis: 0 }, { label: "Y", axis: 1 }, { label: "Z", axis: 2 },
        { label: "Rx", axis: 3 }, { label: "Ry", axis: 4 }, { label: "Rz", axis: 5 }
    ]

    Flow {
        Layout.fillWidth: true
        spacing: 6
        Text { text: qsTr("Step"); color: Theme.textDim; height: 24; verticalAlignment: Text.AlignVCenter }
        Repeater {
            model: [0.001, 0.01, 0.05]
            delegate: Chip {
                required property var modelData
                text: Math.round(modelData * 1000) + " " + qsTr("mm")
                checked: panel.linearStep === modelData
                onClicked: panel.linearStep = modelData
            }
        }
        Item { width: 6; height: 1 }
        Repeater {
            model: [1, 5, 15]
            delegate: Chip {
                required property var modelData
                text: modelData + "°"
                checked: panel.angularStep === modelData
                onClicked: panel.angularStep = modelData
            }
        }
    }

    GridLayout {
        columns: 6
        columnSpacing: 4
        rowSpacing: 4
        Layout.fillWidth: true
        Repeater {
            model: panel.axes
            delegate: RowLayout {
                id: axisRow
                required property var modelData
                Layout.columnSpan: 2
                spacing: 2
                Text {
                    text: modelData.label
                    color: [Theme.axisX, Theme.axisY, Theme.axisZ][modelData.axis % 3]
                    font.bold: true
                    Layout.preferredWidth: 22
                }
                Repeater {
                    model: [-1, 1]
                    delegate: Button {
                        required property var modelData
                        readonly property int axis: axisRow.modelData.axis
                        text: modelData < 0 ? "−" : "+"
                        font.pixelSize: 16
                        leftPadding: 0
                        rightPadding: 0
                        implicitWidth: 40
                        implicitHeight: 36
                        enabled: robot.ready && !robot.busy && !robot.playing
                        onClicked: robot.jogTcp(axis, modelData * (axis < 3 ? panel.linearStep : panel.angularStep))
                    }
                }
            }
        }
    }
}
