import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts

ApplicationWindow {
    id: window
    width: 1400
    height: 940
    minimumWidth: 1000
    minimumHeight: 640
    visible: true
    title: "ManipStudio"
    color: Theme.window

    Material.theme: Material.Dark
    Material.accent: Theme.accent
    Material.primary: Theme.panel

    property string lastError: ""

    Connections {
        target: robot
        function onErrorOccurred(message) { window.lastError = message }
        function onBusyChanged() { if (robot.busy) window.lastError = "" }
    }

    header: ToolBar {
        Material.background: Theme.panel
        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: Theme.gap
            anchors.rightMargin: Theme.gap
            Text { text: "ManipStudio"; color: Theme.text; font.pixelSize: 18; font.bold: true }
            Text {
                text: robot.dataDir
                color: Theme.textDim
                elide: Text.ElideMiddle
                Layout.fillWidth: true
                Layout.leftMargin: Theme.gap
            }
            Rectangle {
                width: 10; height: 10; radius: 5
                color: !robot.ready ? Theme.warn : robot.busy ? Theme.accent : Theme.good
            }
            Chip {
                text: qsTr("Settings")
                enabled: robot.ready
                checked: settingsPanel.opened
                onClicked: settingsPanel.open()
                Layout.rightMargin: Theme.gap
            }
            Row {
                spacing: 4
                Layout.rightMargin: Theme.gap
                Repeater {
                    model: languages.languages
                    delegate: Chip {
                        required property var modelData
                        text: modelData.code.toUpperCase()
                        checked: languages.language === modelData.code
                        onClicked: languages.language = modelData.code
                    }
                }
            }
            Text {
                text: !robot.ready ? qsTr("loading") : robot.busy ? qsTr("planning") : qsTr("ready")
                color: Theme.textDim
            }
        }
    }

    RowLayout {
        anchors.fill: parent
        anchors.margins: Theme.gap
        spacing: Theme.gap

        ScrollView {
            Layout.preferredWidth: 360
            Layout.fillHeight: true
            contentWidth: availableWidth
            clip: true
            ColumnLayout {
                width: parent.width
                spacing: Theme.gap
                JointPanel { Layout.fillWidth: true }
                MovePanel { Layout.fillWidth: true }
            }
        }

        ColumnLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: Theme.gap
            RobotView {
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.minimumHeight: 280
            }
            ResultsPanel {
                Layout.fillWidth: true
                Layout.preferredHeight: 300
                Layout.minimumHeight: 220
            }
        }

        ColumnLayout {
            Layout.preferredWidth: 260
            Layout.maximumWidth: 260
            Layout.fillHeight: true
            spacing: Theme.gap
            PosePanel { Layout.fillWidth: true }
            PeaksPanel { Layout.fillWidth: true }
            Item { Layout.fillHeight: true }
        }
    }

    SettingsPanel { id: settingsPanel; objectName: "settingsPanel" }

    footer: Rectangle {
        color: Theme.panel
        implicitHeight: footerRow.implicitHeight + 12
        RowLayout {
            id: footerRow
            anchors.fill: parent
            anchors.leftMargin: Theme.gap
            anchors.rightMargin: Theme.gap
            spacing: Theme.gap
            Text {
                text: window.lastError !== "" ? window.lastError : robot.status
                color: window.lastError !== "" ? Theme.bad : Theme.textDim
                elide: Text.ElideRight
                Layout.fillWidth: true
            }
            Text {
                visible: robot.sampleCount > 0
                text: (robot.progress * robot.duration).toFixed(2) + " / " + robot.duration.toFixed(2) + qsTr(" s")
                color: Theme.textDim
                font.family: Theme.mono
            }
            ProgressBar {
                Layout.preferredWidth: 200
                value: robot.progress
                indeterminate: robot.busy
            }
        }
    }
}
