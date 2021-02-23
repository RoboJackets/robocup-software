


class InfoPlaySelector(stp.eval.IPlaySelector):

    class PropT:
        last_switch_time: float
        last_play: stp.Play.IPlay

    @staticmethod
    def tick(world_state: rc.WorldState, playbook: IPlaybook, prev_props: Optional[PropT]) -> (stp.Play.IPlay, PropT):

        for a, b in playbook.get_info_plays():
            print(a)
            print(b)
    

