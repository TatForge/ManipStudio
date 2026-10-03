import QtQuick
import QtQuick.Controls

// Compact numeric input. Always a dot as decimal separator and exponents
// allowed (inertia values like 2e-4), whatever the UI language.
TextField {
    id: field
    property real value: 0
    signal edited(real value)

    implicitWidth: 72
    horizontalAlignment: TextInput.AlignRight
    font.family: Theme.mono
    font.pixelSize: 13
    selectByMouse: true
    // 10 significant digits hide float noise such as 99.99999999999999
    function format(v) { return String(+Number(v).toPrecision(10)) }
    text: format(value)
    validator: RegularExpressionValidator { regularExpression: /^[-+]?(\d+\.?\d*|\.\d+)([eE][-+]?\d+)?$/ }
    color: acceptableInput ? Theme.text : Theme.bad

    onEditingFinished: {
        const v = Number(text)
        if (!isNaN(v) && v !== value)
            field.edited(v)
        else
            text = format(value)
    }
}
