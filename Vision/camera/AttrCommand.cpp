#include "AttrCommand.moc"

AttrCommand::AttrCommand(tPvHandle cam, const char *name): AttrBase(cam, name)
{
    setText("Execute");
    connect(this, SIGNAL(clicked()), SLOT(run()));
}

void AttrCommand::run()
{
    printf("Run command \"%s\"\n", name);
    tPvErr err = PvCommandRun(cam, name);
    if (err)
    {
        printf("    err %d\n", err);
    }
}
