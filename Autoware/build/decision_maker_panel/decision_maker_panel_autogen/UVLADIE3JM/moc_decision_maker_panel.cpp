/****************************************************************************
** Meta object code from reading C++ file 'decision_maker_panel.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/autoware/visualization/decision_maker_panel/src/decision_maker_panel.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'decision_maker_panel.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_autoware_rviz_debug__DecisionMakerPanel_t {
    QByteArrayData data[5];
    char stringdata0[70];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_autoware_rviz_debug__DecisionMakerPanel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_autoware_rviz_debug__DecisionMakerPanel_t qt_meta_stringdata_autoware_rviz_debug__DecisionMakerPanel = {
    {
QT_MOC_LITERAL(0, 0, 39), // "autoware_rviz_debug::Decision..."
QT_MOC_LITERAL(1, 40, 9), // "sendTopic"
QT_MOC_LITERAL(2, 50, 0), // ""
QT_MOC_LITERAL(3, 51, 4), // "text"
QT_MOC_LITERAL(4, 56, 13) // "sendEmergency"

    },
    "autoware_rviz_debug::DecisionMakerPanel\0"
    "sendTopic\0\0text\0sendEmergency"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_autoware_rviz_debug__DecisionMakerPanel[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   24,    2, 0x0a /* Public */,
       4,    0,   27,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void,

       0        // eod
};

void autoware_rviz_debug::DecisionMakerPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        DecisionMakerPanel *_t = static_cast<DecisionMakerPanel *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->sendTopic((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 1: _t->sendEmergency(); break;
        default: ;
        }
    }
}

const QMetaObject autoware_rviz_debug::DecisionMakerPanel::staticMetaObject = {
    { &rviz::Panel::staticMetaObject, qt_meta_stringdata_autoware_rviz_debug__DecisionMakerPanel.data,
      qt_meta_data_autoware_rviz_debug__DecisionMakerPanel,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *autoware_rviz_debug::DecisionMakerPanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *autoware_rviz_debug::DecisionMakerPanel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_autoware_rviz_debug__DecisionMakerPanel.stringdata0))
        return static_cast<void*>(this);
    return rviz::Panel::qt_metacast(_clname);
}

int autoware_rviz_debug::DecisionMakerPanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rviz::Panel::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
