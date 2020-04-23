(lang dune 1.11)
(name pt)

(generate_opam_files true)
(source (github nvgrw/prob-tool))
(license PROPRIETARY)
(authors "Niklas Vangerow")
(maintainers "n@vangerow.net")

(package
 (name pt)
 (synopsis "Probabilistic Analysis Tool")
 (description "Probabilistic Analysis Tool for Imperial College London Final Year Project")
 (depends
  (ocaml (= 4.07.1))
  # (core (= v0.12.4))
  # (ppx_deriving (= 4.4))
  (llvm (= 10.0.0))
  # (ppx_import (>= 1.6.2))
  # (ppx_blob (= 0.7.0)))
)
