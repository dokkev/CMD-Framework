"""Any mechanism plugs into the optimizer: a toy one runs, saves and reloads."""

from __future__ import annotations

import dataclasses
import json
from types import MappingProxyType

import pytest

from core.optimization import mechanism as registry
from core.optimization.mechanism import Mechanism, Objective, SummaryColumn
from core.optimization.optimizer import OPTIMIZERS, OptimizerSettings, portfolio, split_settings
from core.optimization.study import load_run, run_study, save_result

SETTINGS = OptimizerSettings(population_size=20, generations=5)


@dataclasses.dataclass(frozen=True)
class ToyConfig:
    upper: float = 10.0
    resolution_mm: float = 0.5
    objectives: tuple[str, ...] | None = None


@dataclasses.dataclass(frozen=True)
class ToyEvaluation:
    x: float
    y: float

    @property
    def g(self) -> tuple[float, ...]:
        return (1.0 - self.x,)  # x >= 1

    @property
    def feasible(self) -> bool:
        return all(value <= 0.0 for value in self.g)


class ToyMechanism(Mechanism):
    name = "toy"
    formulation = "toy-v1"
    variable_names = ("x_mm", "y_mm")
    constraint_names = ("x_at_least_1",)
    objectives = MappingProxyType({
        "transmission": Objective(lambda e: e.x, "x_mm", "x", "best_transmission", False),
        "curl": Objective(lambda e: -e.y + e.x, "y_mm", "y", "best_curl", True),
    })
    summary = (SummaryColumn("x", 5, lambda r: f"{float(r['x_mm']):.1f}"),)

    def config_from_mapping(self, values):
        return ToyConfig(**values)

    def config_to_json(self, config):
        return dataclasses.asdict(config)

    def config_from_json(self, values):
        return ToyConfig(**values)

    def search_box(self, config):
        return (0.0, 0.0), (config.upper, config.upper)

    def evaluate(self, x, config):
        if x[1] > 9.0:  # describes no mechanism
            return None
        return ToyEvaluation(*x)

    def report(self, evaluation):
        return {"x_mm": evaluation.x, "y_mm": evaluation.y}


MECHANISM = ToyMechanism()


def test_a_new_mechanism_runs_saves_and_reloads(tmp_path, monkeypatch) -> None:
    monkeypatch.setitem(registry.MECHANISMS, "toy", __name__)
    assert registry.mechanism("toy") is MECHANISM

    result = run_study(MECHANISM, ToyConfig(), SETTINGS)
    assert result.pareto
    assert all(c.evaluation.x >= 1.0 and c.evaluation.y <= 9.0 for c in result.pareto)
    xs = [c.evaluation.x for c in result.pareto]
    assert xs == sorted(xs)  # by the first objective

    run = load_run(save_result(result, tmp_path / "run"))
    assert run.mechanism is MECHANISM and run.current
    assert run.objectives == ("transmission", "curl")
    assert run.config == ToyConfig()
    assert run.settings == SETTINGS
    assert json.loads((tmp_path / "run" / "run.json").read_text())["mechanism"] == "toy"
    rows = run.candidates()
    assert list(rows[0])[:3] == ["candidate", "x_mm", "y_mm"]
    roles = MECHANISM.seed_roles(
        [MECHANISM.objective_values(c.evaluation, run.objectives) for c in result.pareto],
        [MECHANISM.report(c.evaluation) for c in result.pareto], run.objectives)
    assert set(roles) == set(MECHANISM.role_order(run.objectives))


def test_unknown_mechanisms_and_objectives_are_named() -> None:
    with pytest.raises(ValueError, match="unknown mechanism"):
        registry.mechanism("spring")
    with pytest.raises(ValueError, match="unknown objectives"):
        MECHANISM.active_objectives(ToyConfig(objectives=("speed",)))


@pytest.mark.parametrize("algorithm", sorted(OPTIMIZERS))
def test_every_optimizer_finds_a_feasible_front(algorithm: str) -> None:
    settings = OptimizerSettings(algorithm=algorithm, population_size=20, generations=5)
    result = run_study(MECHANISM, ToyConfig(), settings)

    assert result.pareto and result.settings.algorithm == algorithm
    assert all(c.evaluation.feasible for c in result.pareto)


def test_the_optimizer_settings_come_out_of_the_config_file() -> None:
    values, settings = split_settings({
        "optimizer": {"algorithm": "smsemoa", "objectives": ["curl"], "generations": 7,
                      "options": {"mutation_eta": 30}},
        "seed": 3, "upper": 5.0,
    })
    assert settings == OptimizerSettings(algorithm="smsemoa", generations=7, seed=3,
                                         options={"mutation_eta": 30})
    # The rest of the table stays with the mechanism.
    assert values == {"optimizer": {"objectives": ["curl"]}, "upper": 5.0}
    with pytest.raises(ValueError, match="unknown optimizer"):
        OptimizerSettings(algorithm="cmaes")
    with pytest.raises(ValueError, match="unknown nsga2 options"):
        run_study(MECHANISM, ToyConfig(), OptimizerSettings(population_size=4, generations=1,
                                                            options={"eta": 1}))


def test_a_portfolio_gives_every_run_its_own_seed() -> None:
    plan = portfolio(OptimizerSettings(seed=3), ("nsga2", "smsemoa"), runs_per_algorithm=2)

    assert [(s.algorithm, s.seed) for s in plan] == [
        ("nsga2", 3), ("smsemoa", 4), ("nsga2", 5), ("smsemoa", 6)]
    with pytest.raises(ValueError, match="at least 1"):
        portfolio(OptimizerSettings(), ("nsga2",), runs_per_algorithm=0)


def test_complementary_runs_merge_into_one_verified_front(tmp_path, monkeypatch) -> None:
    from core.optimization.seeds import select_seeds

    monkeypatch.setitem(registry.MECHANISMS, "toy", __name__)
    plan = portfolio(SETTINGS, sorted(OPTIMIZERS))
    runs = [load_run(save_result(run_study(MECHANISM, ToyConfig(), each), tmp_path / f"run_{i}"))
            for i, each in enumerate(plan)]
    selection = select_seeds(runs)

    assert [count[1] for count in selection.counts] == [len(run.candidates()) for run in runs]
    assert {row["algorithm"] for row in selection.rows} <= set(OPTIMIZERS)
    roles = [row["seed_role"] for row in selection.rows]
    assert set(MECHANISM.role_order(runs[0].objectives)) <= {r for role in roles for r in role.split(";")}
    # Merged candidates are mutually non-dominated.
    points = [(row["x_mm"], row["x_mm"] - row["y_mm"]) for row in selection.rows]
    assert not any(a[0] <= b[0] and a[1] <= b[1] and a != b for a in points for b in points)

    other = dataclasses.replace(ToyConfig(), upper=8.0)
    odd = load_run(save_result(run_study(MECHANISM, other, SETTINGS), tmp_path / "odd"))
    with pytest.raises(ValueError, match="differs from"):
        select_seeds([runs[0], odd])
