from typing import Type

import pytest
from stp.utils.typed_key_dict import TypedKey, TypedKeyDict, ValueConcreteT


class Interface:
    def __init__(self, name: str):
        self.name = name

    def __eq__(self, other) -> bool:
        if not isinstance(other, Interface):
            return False

        return self.name == other.name


class ConcreteA(Interface): ...


class ConcreteB(Interface): ...


class Key(TypedKey[ValueConcreteT]):
    def __init__(self, str_val: str, cls: Type[ValueConcreteT]):
        self.str_val = str_val
        super().__init__(cls)

    def __eq__(self, other) -> bool:
        if not isinstance(other, Key):
            return False

        return self.str_val == other.str_val

    def __hash__(self) -> int:
        return hash(self.str_val)


class TKDictFixture:
    def __init__(self):
        # Create a bunch of keys.
        self.key_a1 = Key("a1", ConcreteA)
        self.key_a2 = Key("a2", ConcreteA)
        self.key_b1 = Key("b1", ConcreteB)
        self.key_b2 = Key("b2", ConcreteB)

        # Create the values.
        self.concrete_a1 = ConcreteA("A1")
        self.concrete_a2 = ConcreteA("A2")
        self.concrete_b1 = ConcreteB("B1")
        self.concrete_b2 = ConcreteB("B2")


def test_tkdict_get_set() -> None:
    """Tests that __getitem__ and __setitem__ work properly."""
    fixture = TKDictFixture()
    tkdict: TypedKeyDict[Interface] = TypedKeyDict()

    # Assign the values.
    tkdict[fixture.key_a1] = fixture.concrete_a1
    tkdict[fixture.key_a2] = fixture.concrete_a2
    tkdict[fixture.key_b1] = fixture.concrete_b1
    tkdict[fixture.key_b2] = fixture.concrete_b2

    # Get the values out. Importantly, the concrete types should be inferred properly.
    out_a1: ConcreteA = tkdict[fixture.key_a1]
    out_a2: ConcreteA = tkdict[fixture.key_a2]
    out_b1: ConcreteB = tkdict[fixture.key_b1]
    out_b2: ConcreteB = tkdict[fixture.key_b2]

    # Check that the dictionary functionality still works.
    assert out_a1 == fixture.concrete_a1
    assert out_a2 == fixture.concrete_a2
    assert out_b1 == fixture.concrete_b1
    assert out_b2 == fixture.concrete_b2


def test_tkdict_get_wrong_type() -> None:
    """Check that using a key that hashes to the same value but has a different
    concrete type results in an exception.
    """
    fixture = TKDictFixture()
    tkdict: TypedKeyDict[Interface] = TypedKeyDict()
    tkdict[fixture.key_a1] = fixture.concrete_a1

    wrong_a1_concrete_b = Key("a1", ConcreteB)

    with pytest.raises(KeyError):
        _: ConcreteB = tkdict[wrong_a1_concrete_b]


def test_tkdict_in_len():
    """Check that __contains__ and __len__ works."""
    fixture = TKDictFixture()
    tkdict: TypedKeyDict[Interface] = TypedKeyDict()

    assert fixture.key_a1 not in tkdict
    tkdict[fixture.key_a1] = fixture.concrete_a1
    assert fixture.key_a1 in tkdict
    assert len(tkdict) == 1

    assert fixture.key_a2 not in tkdict
    tkdict[fixture.key_a2] = fixture.concrete_a2
    assert fixture.key_a2 in tkdict
    assert len(tkdict) == 2


def test_tkdict_del_len():
    """Check that __del__ works."""
    fixture = TKDictFixture()
    tkdict: TypedKeyDict[Interface] = TypedKeyDict()

    # Assign the values.
    tkdict[fixture.key_a1] = fixture.concrete_a1
    tkdict[fixture.key_a2] = fixture.concrete_a2
    tkdict[fixture.key_b1] = fixture.concrete_b1
    tkdict[fixture.key_b2] = fixture.concrete_b2

    assert len(tkdict) == 4

    # Delete values.
    del tkdict[fixture.key_a1]
    assert len(tkdict) == 3
    assert fixture.key_a1 not in tkdict

    # Check that a KeyError is raised if we try and get it.
    with pytest.raises(KeyError):
        _: ConcreteA = tkdict[fixture.key_a1]

    # Delete values.
    del tkdict[fixture.key_b1]
    assert len(tkdict) == 2
    assert fixture.key_b1 not in tkdict

    # Check that a KeyError is raised if we try and get it.
    with pytest.raises(KeyError):
        _: ConcreteB = tkdict[fixture.key_b1]


def test_tkdict_keys_values_items():
    """Checks that .keys(), .values() and .items() work as expected."""
    fixture = TKDictFixture()
    tkdict: TypedKeyDict[Interface] = TypedKeyDict()

    # Assign the values.
    tkdict[fixture.key_a1] = fixture.concrete_a1
    tkdict[fixture.key_a2] = fixture.concrete_a2
    tkdict[fixture.key_b1] = fixture.concrete_b1
    tkdict[fixture.key_b2] = fixture.concrete_b2

    # Test .keys().
    expected_keys = [
        fixture.key_a1,
        fixture.key_a2,
        fixture.key_b1,
        fixture.key_b2,
    ]
    assert all([a == b for a, b in zip(tkdict.keys(), expected_keys)])

    # Test .values().
    expected_values = [
        fixture.concrete_a1,
        fixture.concrete_a2,
        fixture.concrete_b1,
        fixture.concrete_b2,
    ]
    assert all([a == b for a, b in zip(tkdict.values(), expected_values)])

    # Test .items().
    assert all([
        a == b for a, b in zip(tkdict.items(), zip(expected_keys, expected_values))
    ])
