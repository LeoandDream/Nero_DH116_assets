def run_once(output):
    """Exactly one caller-authorized attempt; errors never retry the rollout."""
    import os
    output = Path(output).resolve()
    if output.exists() or not output.is_relative_to(ROOT) or os.environ.get('PYTHONPATH') or os.environ.get('PYTHONHOME'):
        raise ValueError('Fresh output in activated checkout required')
    rules, _ = load_contact_policy()
    output.mkdir(parents=True)
    write(output/'attempt-claim.json', {'authorization': AUTHORIZATION, 'baseline': BASELINE,
        'attempt_budget': 1, 'automatic_retries': 0, 'right_frozen_research_rollout': True})
    sys.path.insert(0, str(HERE))
    import white_bowl_exploratory as previous
    import white_bowl_run as legacy
    input_audit = validate_checkout(previous, ROOT)
    registration = previous.protocol(ROOT)
    registration.update(_close149_registration)
    registration.update(authorization=AUTHORIZATION, baseline_commit=BASELINE,
        experiment='issue5_close149_left_drive_profile_v1_once', research_policy_sha256=POLICY_SHA,
        research_rule_version=rules['version'], prior_proposal_status_unchanged=rules['status'],
        online_use_authorized_by=AUTHORIZATION, formal_2mm_and_historical_FAIL='UNCHANGED_SEPARATE_CHANNEL',
        compiler_step_scope='Native compiler self-tests are setup of this authorized attempt, not included in recorded trajectory-step count; no claim of absolute native mj_step instrumentation.')
    write(output/'registered-protocol.json', registration)
    sources = {str(p.relative_to(ROOT)): digest(p) for p in
               (Path(__file__), _close149_source, _profile_path, _patch_path, _research_source, POLICY_DIR/'research-policy.json', POLICY_DIR/'shadow.py', HERE/'white_bowl_run.py')}
    write(output/'input-audit.json', {'previous_inputs': input_audit, 'new_source_sha256': sources})
    observations = Observations(output)
    policy = ResearchPolicy(rules)
    # Construct an explicit policy dependency with the same original mechanics
    # and deferred provenance functions. The old modules/files remain untouched.
    module = types.ModuleType('issue5_research_dependency')
    for key in POLICY_ATTRIBUTES:
        setattr(module, key, getattr(previous, key))
    module.AUTHORIZATION = AUTHORIZATION
    def validate_external(root, path):
        # The historical wrapper also hashes private generated outputs. Use its
        # existing resource/physics validator directly; no old runs are read.
        result = previous.F.validate_external_preflight(root, path)
        return {**result, 'protocol': registration}
    module.validate_external_preflight = validate_external
    original_write = legacy.write
    def record(path, value):
        if Path(path).name == 'frozen-before-first-step.json':
            frozen = read(ROOT/previous.OLD_RUN/'frozen-before-first-step.json')
            for key in ('qpos', 'qvel', 'ctrl', 'controller', 'criteria', 'native_scene'):
                if value[key] != frozen[key]:
                    raise ValueError('Frozen Reset/controller/criteria differ: '+key)
            if read(output/'schedule.json') != _close149_schedule:
                raise ValueError('FrozenClose149actions differ')
            value.update(research_policy_sha256=POLICY_SHA, online_use_authorized_by=AUTHORIZATION,
                         new_source_sha256=sources, baseline_commit=BASELINE)
        if Path(path).name == 'result.json':
            value.update(experiment=registration['experiment'], authorization=AUTHORIZATION,
                A='HISTORICAL_STATIC_RESULTS_PRESERVED_NOT_NEW_ACCEPTANCE',
                baseline_commit=BASELINE, research_policy_sha256=POLICY_SHA, research_rule_version=rules['version'],
                online_use_authorized_by=AUTHORIZATION, research_rollout_status=value['execution'],
                legacy_first_failure=policy.first_legacy, research_first_failure=policy.first_research,
                formal_2mm_and_historical_FAIL='UNCHANGED', observation_frames=observations.frames,
                original_rule_metadata_unchanged=True,
                new_source_sha256_after={p: digest(ROOT/p) for p in sources},
                compiler_step_scope=registration['compiler_step_scope'])
            if value['new_source_sha256_after'] != sources:
                value.update(B='FAIL', input_integrity='CHANGED_NEW_SOURCE')
        original_write(path, value)
    legacy.write = record
    try:
        execute = adapted_executor(legacy, policy, observations, output)
        with selected_policy(module):
            code = execute(ROOT/previous.PLAN, None, output, frozen78=True,
                wrist_conditional=ROOT/'examples/right_hand/evidence/d3_wrist_conditional/preflight/summary.json', finger22=True)
    finally:
        legacy.write = original_write
        observations.close()
    # Video encoding is postprocessing existing images, never a retry.
    try:
        observations.videos()
    except Exception as error:
        write(output/'video-error.json', {'error': str(error), 'physics_retry': False})
    return code
