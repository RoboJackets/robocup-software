#include "AttrFloat.moc"

#include <QDoubleValidator>

AttrFloat::AttrFloat(tPvHandle cam, const char *name): AttrBase(cam, name)
{
    tPvFloat32 min = 0, max = 0;
    PvAttrRangeFloat32(cam, name, &min, &max);
    
    setValidator(new QDoubleValidator(min, max, 3, this));
    
    connect(this, SIGNAL(returnPressed()), SLOT(change()));
}

void AttrFloat::reset()
{
    tPvFloat32 cur = 0;
    PvAttrFloat32Get(cam, name, &cur);
    setText(QString::number(cur));
}

void AttrFloat::change()
{
    if (!isEnabled())
    {
        return;
    }
 
    tPvFloat32 value = text().toFloat();
    printf("Change \"%s\" to %f\n", name, value);
    tPvErr err = PvAttrFloat32Set(cam, name, value);
    if (err)
    {
        printf("    err %d\n", err);
    }
}
