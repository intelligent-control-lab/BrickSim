"""Tests for brick-part transform setters."""

import pytest
from pxr import Gf, Usd, UsdGeom

from bricksim.mdp.brick_part import set_brick_part_xform


def test_set_brick_part_xform_edits_existing_translate_and_orient_ops():
    """Existing translate/orient ops should be updated without resetting scale."""
    stage = Usd.Stage.CreateInMemory()
    prim = UsdGeom.Xform.Define(stage, "/Brick").GetPrim()
    xformable = UsdGeom.Xformable(prim)
    xformable.AddTranslateOp().Set(Gf.Vec3d(1.0, 2.0, 3.0))
    xformable.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Quatd(1.0, 0.0, 0.0, 0.0)
    )
    xformable.AddScaleOp().Set(Gf.Vec3d(2.0, 3.0, 4.0))
    op_names_before = [op.GetName() for op in xformable.GetOrderedXformOps()]

    set_brick_part_xform(
        prim,
        translation=(0.1, 0.2, 0.3),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )

    transform = Gf.Transform(UsdGeom.Xformable(prim).GetLocalTransformation())
    rotation = transform.GetRotation().GetQuat()
    imaginary = rotation.GetImaginary()
    op_names_after = [op.GetName() for op in xformable.GetOrderedXformOps()]
    assert op_names_after == op_names_before
    assert tuple(float(value) for value in transform.GetTranslation()) == pytest.approx(
        (0.1, 0.2, 0.3)
    )
    assert (
        float(rotation.GetReal()),
        float(imaginary[0]),
        float(imaginary[1]),
        float(imaginary[2]),
    ) == pytest.approx((0.0, 0.0, 0.0, 1.0))
    assert tuple(float(value) for value in transform.GetScale()) == pytest.approx(
        (2.0, 3.0, 4.0)
    )


def test_set_brick_part_xform_resets_when_canonical_ops_are_missing():
    """Missing canonical ops should fall back to authoring the full xform stack."""
    stage = Usd.Stage.CreateInMemory()
    prim = UsdGeom.Xform.Define(stage, "/Brick").GetPrim()
    UsdGeom.Xformable(prim).AddScaleOp().Set(Gf.Vec3d(2.0, 3.0, 4.0))

    set_brick_part_xform(
        prim,
        translation=(0.1, 0.2, 0.3),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )

    transform = Gf.Transform(UsdGeom.Xformable(prim).GetLocalTransformation())
    rotation = transform.GetRotation().GetQuat()
    imaginary = rotation.GetImaginary()
    op_names = [op.GetName() for op in UsdGeom.Xformable(prim).GetOrderedXformOps()]
    assert "xformOp:translate" in op_names
    assert "xformOp:orient" in op_names
    assert tuple(float(value) for value in transform.GetTranslation()) == pytest.approx(
        (0.1, 0.2, 0.3)
    )
    assert (
        float(rotation.GetReal()),
        float(imaginary[0]),
        float(imaginary[1]),
        float(imaginary[2]),
    ) == pytest.approx((0.0, 0.0, 0.0, 1.0))
    assert tuple(float(value) for value in transform.GetScale()) == pytest.approx(
        (2.0, 3.0, 4.0)
    )
