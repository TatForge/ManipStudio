import QtQuick

// Compact toggle: filled with `tint` when checked, outlined otherwise.
Rectangle {
    id: chip
    property alias text: label.text
    property color tint: Theme.accent
    property bool checked: false
    signal clicked()

    implicitWidth: label.implicitWidth + 16
    implicitHeight: 24
    radius: height / 2
    color: checked ? tint : "transparent"
    border.color: checked ? tint : Theme.panelBorder
    opacity: enabled ? 1 : 0.4

    Text {
        id: label
        anchors.centerIn: parent
        color: chip.checked ? Theme.window : chip.tint
        font.pixelSize: 12
        font.bold: chip.checked
    }
    MouseArea {
        anchors.fill: parent
        cursorShape: Qt.PointingHandCursor
        onClicked: chip.clicked()
    }
}
