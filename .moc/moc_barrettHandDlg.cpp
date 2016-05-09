/****************************************************************************
** Meta object code from reading C++ file 'barrettHandDlg.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../ui/barrettHandDlg.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'barrettHandDlg.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_BarrettHandDlg[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      16,   15,   15,   15, 0x0a,
      33,   15,   15,   15, 0x0a,
      58,   15,   15,   15, 0x0a,
      83,   15,   15,   15, 0x0a,
     111,   15,   15,   15, 0x0a,
     134,   15,   15,   15, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_BarrettHandDlg[] = {
    "BarrettHandDlg\0\0initializeHand()\0"
    "simulationFromRealHand()\0"
    "realHandFromSimulation()\0"
    "toggleContinuousOperation()\0"
    "smoothButton_clicked()\0"
    "initSpreadButton_clicked()\0"
};

void BarrettHandDlg::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        BarrettHandDlg *_t = static_cast<BarrettHandDlg *>(_o);
        switch (_id) {
        case 0: _t->initializeHand(); break;
        case 1: _t->simulationFromRealHand(); break;
        case 2: _t->realHandFromSimulation(); break;
        case 3: _t->toggleContinuousOperation(); break;
        case 4: _t->smoothButton_clicked(); break;
        case 5: _t->initSpreadButton_clicked(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData BarrettHandDlg::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject BarrettHandDlg::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_BarrettHandDlg,
      qt_meta_data_BarrettHandDlg, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &BarrettHandDlg::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *BarrettHandDlg::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *BarrettHandDlg::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_BarrettHandDlg))
        return static_cast<void*>(const_cast< BarrettHandDlg*>(this));
    if (!strcmp(_clname, "Ui::BarrettHandDlgUI"))
        return static_cast< Ui::BarrettHandDlgUI*>(const_cast< BarrettHandDlg*>(this));
    return QDialog::qt_metacast(_clname);
}

int BarrettHandDlg::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
