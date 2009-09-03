#include "AttrEnum.moc"

AttrEnum::AttrEnum(tPvHandle cam, const char *name): AttrBase(cam, name)
{
    char buf[1024];
    
    PvAttrRangeEnum(cam, name, buf, sizeof(buf), NULL);
    QStringList items = QString(buf).split(",");
    addItems(items);

    connect(this, SIGNAL(activated(int)), SLOT(change()));
}

void AttrEnum::reset()
{
}

void AttrEnum::change()
{
    if (!isEnabled())
    {
        return;
    }
 
    char *value = strdup(qPrintable(currentText()));
    printf("Change \"%s\" to \"%s\"\n", name, value);
    tPvErr err = PvAttrEnumSet(cam, name, value);
    if (err)
    {
        printf("    err %d\n", err);
    }
    free(value);
}
