import QtQuick
import QtQuick.Layouts

// Peak |tau| of every joint over the last trajectory against its motor limit.
Card {
    title: qsTr("Пиковые моменты")

    Text {
        visible: robot.peaks.length === 0
        text: qsTr("нет траектории")
        color: Theme.textDim
    }

    Repeater {
        model: robot.peaks
        delegate: ColumnLayout {
            required property var modelData
            required property int index
            Layout.fillWidth: true
            spacing: 2

            readonly property color level: modelData.exceeded ? Theme.bad : modelData.ratio > 0.8 ? Theme.warn : Theme.good

            RowLayout {
                Layout.fillWidth: true
                Text { text: "q" + (index + 1); color: Theme.jointColors[index]; font.bold: true; Layout.preferredWidth: 24 }
                Text {
                    Layout.fillWidth: true
                    horizontalAlignment: Text.AlignRight
                    text: modelData.peak.toFixed(1) + " / " + modelData.limit.toFixed(0) + qsTr(" Н·м")
                    color: Theme.text
                    font.family: Theme.mono
                }
                Text {
                    // one decimal near the limit, so 100.2 % does not read as 100 %
                    text: (modelData.ratio * 100).toFixed(Math.abs(modelData.ratio - 1) < 0.05 ? 1 : 0) + "%"
                    color: level
                    font.family: Theme.mono
                    horizontalAlignment: Text.AlignRight
                    Layout.preferredWidth: 52
                }
            }
            Rectangle {
                Layout.fillWidth: true
                height: 4
                radius: 2
                color: Theme.panelBorder
                Rectangle {
                    width: parent.width * Math.min(modelData.ratio, 1)
                    height: parent.height
                    radius: 2
                    color: level
                }
            }
        }
    }

    Text {
        visible: robot.trajectoryError !== ""
        Layout.fillWidth: true
        wrapMode: Text.Wrap
        text: qsTr("Траектория отклонена: ") + robot.trajectoryError
        color: Theme.bad
        font.pixelSize: 12
    }
}
