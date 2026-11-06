"""Contact query retained locally after playground removed its collision module."""
import jax.numpy as jp


def geoms_colliding(data, geom1, geom2):
    contact = getattr(data, "_impl", data).contact
    pair = contact.geom
    match = (((pair[:, 0] == geom1) & (pair[:, 1] == geom2))
             | ((pair[:, 0] == geom2) & (pair[:, 1] == geom1)))
    return jp.any(match & (contact.dist <= 0))
