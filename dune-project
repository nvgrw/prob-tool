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
  (llvm (= 10.0.0))
  (owl (= 0.8.0)))
)