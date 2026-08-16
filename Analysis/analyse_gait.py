import analysis_functions as af

behaviour = "kb+height_+land_walk++transit"


af.load_data()

af.plot_swimming(behaviour)
af.plot_walking(behaviour)
af.plot_target_errors(behaviour)


af.get_behaviour_df(behaviour)