import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

// Jog: one slider per joint, bounded by the joint limits from the settings file.
Card {
    title: qsTr("Joints")

    Repeater {
        model: robot.limits.length
        delegate: RowLayout {
            required property int index
            readonly property var limit: robot.limits[index]
            Layout.fillWidth: true
            spacing: 8

            Text {
                text: "q" + (index + 1)
                color: Theme.text
                font.family: Theme.mono
                Layout.preferredWidth: 24
            }
            Slider {
                Layout.fillWidth: true
                from: limit.min
                to: limit.max
                enabled: robot.ready && !robot.busy
                value: robot.joints[index]
                onMoved: robot.setJoint(index, value)
            }
            Text {
                text: robot.joints[index].toFixed(1) + "°"
                color: Theme.text
                font.family: Theme.mono
                horizontalAlignment: Text.AlignRight
                Layout.preferredWidth: 56
            }
        }
    }

    RowLayout {
        Layout.fillWidth: true
        Button {
            text: qsTr("Zero")
            flat: true
            enabled: robot.ready && !robot.busy
            onClicked: robot.setJoints(robot.zeroPose())
        }
        Button {
            text: qsTr("Folded")
            flat: true
            enabled: robot.ready && !robot.busy
            onClicked: robot.setJoints(robot.internalPose())
        }
    }
}
