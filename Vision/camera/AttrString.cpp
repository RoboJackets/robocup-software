#include "AttrString.moc"

AttrString::AttrString(tPvHandle cam, const char *name): AttrBase(cam, name)
{
    connect(this, SIGNAL(returnPressed()), SLOT(change()));
}

void AttrString::reset()
{
    PvAttrStringGet(cam, name, buf, sizeof(buf), 0);
    setText(buf);
}

void AttrString::change()
{
    if (!isEnabled())
    {
        return;
    }

    char *value = strdup(qPrintable(text()));
    printf("Change \"%s\" to \"%s\"\n", name, value);
    tPvErr err = PvAttrStringSet(cam, name, value);
    if (err)
    {
        printf("    err %d\n", err);
    }
    free(value);
}
