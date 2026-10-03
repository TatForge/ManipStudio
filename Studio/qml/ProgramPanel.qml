import QtQuick
import QtQuick.Controls
import QtQuick.Dialogs
import QtQuick.Layouts

// The program: steps added from the motion panel, run one after another as
// a single trajectory; can be reordered, saved and loaded (JSON).
Card {
    id: panel
    title: qsTr("Program")

    function summary(step) {
        const f = (v, d) => Number(v).toFixed(d)
        if (step.type === "J")
            return step.joints.map(v => f(v, 0)).join(", ") + "°"
        const p = step.pose
        return "(" + f(p[0], 3) + ", " + f(p[1], 3) + ", " + f(p[2], 3) + ")"
    }

    Text {
        visible: robot.program.length === 0
        Layout.fillWidth: true
        wrapMode: Text.Wrap
        text: qsTr("Empty. Set a target in the motion panel and press «To program».")
        color: Theme.textDim
        font.pixelSize: 12
    }

    Repeater {
        model: robot.program
        delegate: RowLayout {
            required property var modelData
            required property int index
            Layout.fillWidth: true
            spacing: 4
            Text { text: (index + 1) + "."; color: Theme.textDim; font.family: Theme.mono; Layout.preferredWidth: 22 }
            Text {
                text: modelData.type
                color: modelData.type === "J" ? Theme.accent : modelData.type === "L" ? Theme.good : Theme.warn
                font.bold: true
                Layout.preferredWidth: 14
            }
            Text {
                text: panel.summary(modelData)
                color: Theme.text
                font.family: Theme.mono
                font.pixelSize: 11
                elide: Text.ElideRight
                // takes what is left, never pushes the buttons out
                Layout.fillWidth: true
                Layout.preferredWidth: 1
            }
            Chip { text: "↑"; tint: Theme.textDim; enabled: index > 0; onClicked: robot.moveStep(index, -1) }
            Chip { text: "↓"; tint: Theme.textDim; enabled: index < robot.program.length - 1; onClicked: robot.moveStep(index, 1) }
            Chip { text: "✕"; tint: Theme.bad; onClicked: robot.removeStep(index) }
        }
    }

    Button {
        Layout.fillWidth: true
        text: qsTr("Run program")
        highlighted: true
        enabled: robot.ready && !robot.busy && robot.program.length > 0
        onClicked: robot.runProgram()
    }
    RowLayout {
        Layout.fillWidth: true
        spacing: 6
        Chip { text: qsTr("Save"); enabled: robot.program.length > 0; onClicked: saveDialog.open() }
        Chip { text: qsTr("Load"); onClicked: loadDialog.open() }
        Item { Layout.fillWidth: true }
        Chip { text: qsTr("Clear"); tint: Theme.bad; enabled: robot.program.length > 0; onClicked: robot.clearProgram() }
    }

    FileDialog {
        id: saveDialog
        title: qsTr("Save program")
        fileMode: FileDialog.SaveFile
        defaultSuffix: "json"
        nameFilters: [qsTr("Program (*.json)")]
        onAccepted: robot.saveProgram(selectedFile)
    }
    FileDialog {
        id: loadDialog
        title: qsTr("Load program")
        fileMode: FileDialog.OpenFile
        nameFilters: [qsTr("Program (*.json)")]
        onAccepted: robot.loadProgram(selectedFile)
    }
}
