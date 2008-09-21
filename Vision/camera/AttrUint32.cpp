#include "AttrUint32.moc"

AttrUint32::AttrUint32(tPvHandle cam, const char *name): AttrBase(cam, name)
{
    tPvUint32 min = 0, max = 0;
    PvAttrRangeUint32(cam, name, &min, &max);
    
    if (max > INT_MAX)
    {
        max = INT_MAX;
    }
    
    setMinimum(min);
    setMaximum(max);
    
    connect(this, SIGNAL(editingFinished()), SLOT(change()));
}

void AttrUint32::reset()
{
    tPvUint32 cur = 0;
    PvAttrUint32Get(cam, name, &cur);
    setValue(cur);
}

void AttrUint32::change()
{
    if (!isEnabled())
    {
        return;
    }
 
    printf("Change \"%s\" to %d\n", name, value());
    tPvErr err = PvAttrUint32Set(cam, name, value());
    if (err)
    {
        printf("    err %d\n", err);
    }
}
