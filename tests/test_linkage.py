from dataclasses import FrozenInstanceError, fields

import pytest

from cmd.morphology import (
    AnchorYZ,
    CouplingAnchorParameters,
    FingerLinkage,
    FingerLinkageParameters,
    MainChainParameters,
)


def make_parameters() -> FingerLinkageParameters:
    return FingerLinkageParameters(
        main_chain=MainChainParameters(
            proximal_length_mm=20.0,
            middle_length_mm=15.0,
            distal_length_mm=10.0,
        ),
        coupling=CouplingAnchorParameters(
            base_anchor=AnchorYZ(y_mm=0.0, z_mm=3.0),
            proximal_anchor=AnchorYZ(y_mm=3.0, z_mm=2.0),
            middle_anchor=AnchorYZ(y_mm=4.0, z_mm=-2.0),
            distal_anchor=AnchorYZ(y_mm=5.0, z_mm=1.0),
        ),
    )


def test_reference_positions_use_the_owning_body_frames() -> None:
    linkage = FingerLinkage(make_parameters())
    reference = linkage.reference_positions

    assert reference.o == AnchorYZ(0.0, 0.0)
    assert reference.a == AnchorYZ(20.0, 0.0)
    assert reference.b == AnchorYZ(35.0, 0.0)
    assert reference.c == AnchorYZ(45.0, 0.0)

    assert reference.op == AnchorYZ(0.0, 3.0)
    assert reference.ap == AnchorYZ(3.0, 2.0)
    assert reference.bp == AnchorYZ(24.0, -2.0)
    assert reference.cp == AnchorYZ(40.0, 1.0)


def test_coupler_lengths_are_derived_from_reference_positions() -> None:
    linkage = FingerLinkage(make_parameters())
    reference = linkage.reference_positions

    assert linkage.coupler_1_length_mm == pytest.approx(
        reference.op.distance_to(reference.bp)
    )
    assert linkage.coupler_2_length_mm == pytest.approx(
        reference.ap.distance_to(reference.cp)
    )
    assert linkage.coupler_1_length_mm == pytest.approx((24.0**2 + 5.0**2) ** 0.5)
    assert linkage.coupler_2_length_mm == pytest.approx((37.0**2 + 1.0**2) ** 0.5)

    parameter_names = {field.name for field in fields(linkage.parameters)}
    assert "coupler_1_length_mm" not in parameter_names
    assert "coupler_2_length_mm" not in parameter_names


def test_parameter_fields_are_only_independent_design_variables() -> None:
    parameters = make_parameters()

    assert {field.name for field in fields(parameters)} == {
        "main_chain",
        "coupling",
    }
    assert {field.name for field in fields(parameters.main_chain)} == {
        "proximal_length_mm",
        "middle_length_mm",
        "distal_length_mm",
    }
    assert {field.name for field in fields(parameters.coupling)} == {
        "base_anchor",
        "proximal_anchor",
        "middle_anchor",
        "distal_anchor",
    }

    assert not hasattr(parameters, "q1")
    assert not hasattr(parameters, "q2")
    assert not hasattr(parameters, "q3")
    assert not hasattr(parameters, "coupler_1_length_mm")
    assert not hasattr(parameters, "coupler_2_length_mm")


def test_planar_convention_has_no_independent_x_coordinate() -> None:
    linkage = FingerLinkage(make_parameters())

    assert linkage.flexion_axis_xyz == (1.0, 0.0, 0.0)
    assert not hasattr(linkage.parameters.coupling.base_anchor, "x_mm")


def test_parameter_dataclasses_are_immutable() -> None:
    parameters = make_parameters()

    with pytest.raises(FrozenInstanceError):
        parameters.main_chain = MainChainParameters(1.0, 1.0, 1.0)

    with pytest.raises(FrozenInstanceError):
        parameters.coupling.base_anchor.y_mm = 1.0


@pytest.mark.parametrize(
    "constructor",
    [
        lambda: MainChainParameters(0.0, 1.0, 1.0),
        lambda: MainChainParameters(1.0, float("inf"), 1.0),
        lambda: AnchorYZ(float("nan"), 0.0),
    ],
)
def test_invalid_physical_or_anchor_values_are_rejected(constructor) -> None:
    with pytest.raises(ValueError):
        constructor()


def test_zero_length_coupler_is_rejected_when_constructing_linkage() -> None:
    parameters = FingerLinkageParameters(
        main_chain=MainChainParameters(20.0, 15.0, 10.0),
        coupling=CouplingAnchorParameters(
            base_anchor=AnchorYZ(24.0, -2.0),
            proximal_anchor=AnchorYZ(3.0, 2.0),
            middle_anchor=AnchorYZ(4.0, -2.0),
            distal_anchor=AnchorYZ(5.0, 1.0),
        ),
    )

    with pytest.raises(ValueError, match="coupler 1"):
        FingerLinkage(parameters)
