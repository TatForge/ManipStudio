import QtQuick
import QtQuick.Layouts

// TCP position and orientation from forward kinematics.
Card {
    title: qsTr("Tool (TCP)")

    readonly property var names: ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
    readonly property var units: [qsTr("m"), qsTr("m"), qsTr("m"), "°", "°", "°"]
    readonly property var colors: [Theme.axisX, Theme.axisY, Theme.axisZ, Theme.axisX, Theme.axisY, Theme.axisZ]

    GridLayout {
        columns: 3
        columnSpacing: 10
        rowSpacing: 4
        Layout.fillWidth: true

        Repeater {
            model: 6
            delegate: Text {
                required property int index
                Layout.row: index
                Layout.column: 0
                text: names[index]
                color: colors[index]
                font.bold: true
            }
        }
        Repeater {
            model: 6
            delegate: Text {
                required property int index
                Layout.row: index
                Layout.column: 1
                Layout.fillWidth: true
                horizontalAlignment: Text.AlignRight
                text: robot.tcp.length === 6 ? robot.tcp[index].toFixed(index < 3 ? 4 : 2) : "—"
                color: Theme.text
                font.family: Theme.mono
                font.pixelSize: 15
            }
        }
        Repeater {
            model: 6
            delegate: Text {
                required property int index
                Layout.row: index
                Layout.column: 2
                text: units[index]
                color: Theme.textDim
            }
        }
    }
}
