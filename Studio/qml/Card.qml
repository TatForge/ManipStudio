import QtQuick
import QtQuick.Layouts

// Titled panel; children go into a column below the title.
Rectangle {
    id: card
    property string title
    default property alias content: column.data

    color: Theme.panel
    border.color: Theme.panelBorder
    radius: Theme.radius
    implicitHeight: column.implicitHeight + titleText.height + 3 * Theme.gap
    implicitWidth: 320

    Text {
        id: titleText
        text: card.title
        color: Theme.textDim
        font.pixelSize: 12
        font.capitalization: Font.AllUppercase
        font.letterSpacing: 1
        anchors { left: parent.left; top: parent.top; margins: Theme.gap }
    }

    ColumnLayout {
        id: column
        spacing: 6
        anchors { left: parent.left; right: parent.right; top: titleText.bottom; margins: Theme.gap }
    }
}
